"""
Reproduce all figures for the Batch Column Generation (B-CG) analysis.

Benchmark results must already exist under Outputs/Phase_2/.

Usage
-----
    python scripts/plot_BCG.py                               # all folders
    python scripts/plot_BCG.py --folders ablation            # one folder
    python scripts/plot_BCG.py --folders ablation multiObj   # several
    python scripts/plot_BCG.py --folders all                 # explicit all
    python scripts/plot_BCG.py --gather-data                 # merge before plotting

Available folders
-----------------
    nbPicks, pruning, truncate, dropPick, dynamic, commit,
    ablation, multiObj, customer_weight, compare, obj_compare
"""

import sys
import os
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import batch_CG_postprocess as postprocess
import constants as c

DEFAULT_PHASE   = "Phase_2"
DEFAULT_FOLDERS = "all"

AVAILABLE_FOLDERS = list(c.FOLDERS.get(DEFAULT_PHASE, {}).keys())


def main():
    parser = argparse.ArgumentParser(
        description="Reproduce B-CG figures from pre-computed benchmark results.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--phase",
        default=DEFAULT_PHASE,
        help=f"Results phase directory (default: {DEFAULT_PHASE}).",
    )
    parser.add_argument(
        "--folders",
        nargs="+",
        default=[DEFAULT_FOLDERS],
        metavar="FOLDER",
        help=(
            f"Folder key(s) to plot, or 'all'. "
            f"Available: {', '.join(AVAILABLE_FOLDERS)}. "
            f"(default: {DEFAULT_FOLDERS})"
        ),
    )
    parser.add_argument(
        "--gather-data",
        action="store_true",
        help="Merge epoch results before plotting.",
    )
    args = parser.parse_args()

    selected = args.folders
    if len(selected) == 1 and selected[0] == "all":
        selected = "all"
    elif len(selected) == 1:
        selected = selected[0]

    postprocess.main(
        name="",
        phase=args.phase,
        selected_folders=selected,
        gather_data=args.gather_data,
    )


if __name__ == "__main__":
    main()
