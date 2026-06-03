#!/usr/bin/env python3
"""Generate reproducible command lines for the DARP C++ executable.

This script does not solve anything itself. It expands experiment configs
(JSON) into shell commands that call the C++ binary, e.g. bin/realtime_DARP.

Example:
  python scripts/make_commands.py \
    --config experiments/A_CG.json \
    --commands-out commands/anytime_paper.txt \
    --output-dir results/anytime_paper
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def _as_list(value: Any) -> List[Any]:
    if value is None:
        return []
    if isinstance(value, list):
        return value
    return [value]


def _split_csv(values: Optional[List[str]]) -> Optional[set[str]]:
    if not values:
        return None
    out: set[str] = set()
    for item in values:
        for piece in str(item).replace(",", " ").split():
            if piece:
                out.add(piece)
    return out or None


def _quote_cmd(parts: Iterable[Any]) -> str:
    return " ".join(shlex.quote(str(p)) for p in parts)


def _discover_instances(repo_root: Path, group_id: str, group: Dict[str, Any]) -> List[str]:
    data_dir = group["data_dir"]
    inst_folder = group["inst_folder"]
    folder = repo_root / data_dir / inst_folder
    if not folder.is_dir():
        print(
            f"[WARN] {group_id}: auto_discover requested but directory not found: {folder}",
            file=sys.stderr,
        )
        return []
    instances = sorted(p.name for p in folder.iterdir() if p.is_dir())
    if not instances:
        print(f"[WARN] {group_id}: no instance subdirectories found in {folder}", file=sys.stderr)
    return instances


def _load_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        cfg = json.load(f)
    if "groups" not in cfg or "runs" not in cfg:
        raise ValueError("Config must contain top-level 'groups' and 'runs'.")
    if not isinstance(cfg["groups"], dict):
        raise ValueError("Config field 'groups' must be an object keyed by group id.")
    if not isinstance(cfg["runs"], list):
        raise ValueError("Config field 'runs' must be a list.")
    return cfg


def generate_commands(
    cfg: Dict[str, Any],
    repo_root: Path,
    exe: str,
    output_dir: str,
    run_filter: Optional[set[str]] = None,
    group_filter: Optional[set[str]] = None,
    scenario_filter: Optional[set[str]] = None,
) -> List[str]:
    groups: Dict[str, Dict[str, Any]] = cfg["groups"]
    commands: List[str] = []

    for run in cfg["runs"]:
        run_id = run.get("id", "")
        if run_filter is not None and run_id not in run_filter:
            continue

        run_groups = [g for g in _as_list(run.get("groups")) if g]
        run_scenarios = [s for s in _as_list(run.get("scenarios")) if s]
        if scenario_filter is not None:
            run_scenarios = [s for s in run_scenarios if s in scenario_filter]
        if not run_scenarios:
            print(f"[WARN] run {run_id}: no scenarios selected; skipping.", file=sys.stderr)
            continue

        main_algo = run.get("main_algo", cfg.get("main_algo"))
        sol_mode = run.get("sol_mode", cfg.get("sol_mode"))
        paramfile = run.get("paramfile", cfg.get("paramfile"))
        if main_algo is None or sol_mode is None or not paramfile:
            raise ValueError(f"Run {run_id!r} must define main_algo, sol_mode, and paramfile.")

        for group_id in run_groups:
            if group_filter is not None and group_id not in group_filter:
                continue
            if group_id not in groups:
                raise KeyError(f"Run {run_id!r} references unknown group {group_id!r}.")
            group = groups[group_id]
            instances = group.get("instances")
            if group.get("auto_discover", False):
                instances = _discover_instances(repo_root, group_id, group)
            instances = [str(x) for x in _as_list(instances)]
            vehicle_counts = [str(x) for x in _as_list(group.get("vehicle_counts"))]
            if not instances or not vehicle_counts:
                print(
                    f"[WARN] run {run_id}, group {group_id}: empty instances or vehicle_counts; skipping.",
                    file=sys.stderr,
                )
                continue

            for scenario in run_scenarios:
                for count in vehicle_counts:
                    for instance in instances:
                        parts = [
                            exe,
                            "--data-dir", group["data_dir"],
                            "--vehicle-folder", group["vehicle_folder"],
                            "--inst-folder", group["inst_folder"],
                            "--instance-name", instance,
                            "--num-vehicles", count,
                            "--vehicle-capacity", group["capacity"],
                            "--main-algo", main_algo,
                            "--sol-mode", sol_mode,
                            "--paramfile", paramfile,
                            "--scenario", scenario,
                            "--output-dir", output_dir,
                            "--initial-state", group["initial_state"],
                        ]
                        commands.append(_quote_cmd(parts))
    return commands


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate DARP experiment command files.")
    parser.add_argument("--config", required=True, type=Path, help="Path to experiments/*.json")
    parser.add_argument("--repo-root", default=".", type=Path, help="Repository root, used for auto-discovery")
    parser.add_argument("--exe", default=os.environ.get("EXE", "../bin/realtime_DARP"), help="C++ executable path")
    parser.add_argument("--output-dir", default=os.environ.get("OUTPUT_DIR", "results"), help="Output directory passed to the C++ code")
    parser.add_argument("--commands-out", type=Path, help="Write commands to this file")
    parser.add_argument("--dry-run", action="store_true", help="Print generated commands to stdout")
    parser.add_argument("--run", action="append", help="Filter by run id. Can be repeated or comma-separated.")
    parser.add_argument("--group", action="append", help="Filter by group id. Can be repeated or comma-separated.")
    parser.add_argument("--scenario", action="append", help="Filter by scenario name. Can be repeated or comma-separated.")

    args = parser.parse_args()
    Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    cfg = _load_config(args.config)
    commands = generate_commands(
        cfg=cfg,
        repo_root=args.repo_root.resolve(),
        exe=args.exe,
        output_dir=args.output_dir,
        run_filter=_split_csv(args.run),
        group_filter=_split_csv(args.group),
        scenario_filter=_split_csv(args.scenario),
    )

    if args.commands_out:
        args.commands_out.parent.mkdir(parents=True, exist_ok=True)
        args.commands_out.write_text("\n".join(commands) + ("\n" if commands else ""), encoding="utf-8")
        print(f"[INFO] wrote {len(commands)} commands to {args.commands_out}")
        if commands:
            print(f"[INFO] example: scripts/run_commands.sh {args.commands_out}")

    if args.dry_run or not args.commands_out:
        for cmd in commands:
            print(cmd)
        if not commands:
            print("[WARN] No commands generated.", file=sys.stderr)
            return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
