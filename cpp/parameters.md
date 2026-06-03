# Parameter Reference

Parameters are read from a JSON file (e.g. `AnyParameters.json` or `BatchParameters.json`)
and a scenario is selected on the command line:

```
--paramfile <name> --scenario <key>
```

Each scenario block sets **only** the keys it needs to override; every other parameter falls
back to the code default listed below. The defaults and JSON key names in this document are
taken directly from `ReadWrite::readParametersJson` (in `ReadWrite.cpp`), and the option
values correspond to the enums in `Types.h`. Where a paper uses a different value for an
experiment, it is noted in the **Notes** column.

**Conventions**
- `LARGE_CONSTANT = 1e7` is used as a sentinel for "∞ / disabled" (see `Types.h`).
- Booleans are written in JSON as `1` (true) / `0` (false).
- Symbols in the *Notes* column refer to the notation used in the two papers
  (B&ndash;CG: *Accelerated Column Generation*; A&ndash;CG: *Anytime Optimization Approach
  for Dynamic DARP*).

---

## Objective Weights

| Key | Type | Default | Description | Notes |
|---|---|---|---|---|
| `Wait_W1` | float | `1.0` | Weight on total passenger waiting time in the objective | &omega;<sub>wait</sub> |
| `Ride_W2` | float | `0.0` | Weight on trip delay (excess ride time = ride time &minus; direct travel time) | &omega;<sub>drop</sub>; papers use `0.5` (best trade-off) and `1.0` |
| `Req_W3` | bool | `0` | Weight each objective term by the number of passengers `q_i` of the request (`1` = enabled) | Appendix G of B&ndash;CG (per-passenger weighting) |
| `Ride_W4` | bool | `0` | Use **total** in-vehicle ride time instead of excess ride time (`1` = enabled) | Jung et al. variant (&lambda;<sub>excess</sub>=0) |
| `Relative_W5` | bool | `0` | Normalize the ride-time term by the direct travel time `t_i` &rarr; relative detour (`1` = enabled) | Pfeiffer &amp; Schulz variant (&lambda;<sub>normal</sub>=`t_i`) |
| `Normal_W6` | bool | `0` | Express/average the objective per served customer (`1` = enabled) | Reported in code as "objective per customer" |

> The default objective (`Wait_W1=1`, `Ride_W2=0`) reproduces the waiting-time-only objective
> of Riley et al. (2019). Setting `Ride_W2=0.5` adds the trip-delay term used throughout both papers.

---

## Model Parameters and Algorithmic Setting

| Key | Type | Default | Description | Notes |
|---|---|---|---|---|
| `alphaParam` | float | `1.5` | Detour factor &alpha;: contributes to max ride time `t_max = max(α·t_i, β + t_i)` | &alpha; |
| `betaParam` | float | `240.0` | Additive ride-time slack &beta; (seconds) in `t_max = max(α·t_i, β + t_i)` | &beta;; Set 2 / shuttle experiments use `300`&ndash;`600` |
| `deltaPram` | float | `420.0` | Base penalty (in equivalent seconds) for an unserved request: `p_i = δ · 2^((τℓ − e_i)/(penaltyL·ℓ))` | &rho; in the papers; shuttle scenarios use `600` |
| `penaltyL` | int | `30` | Epoch-length term used in the unserved-request penalty growth formula | denominator scaling in `p_i` |
| `epochLength` | float | `30.0` | Length of a fixed epoch / time slot (seconds); used in `DYNAMIC` mode (B&ndash;CG) | &ell; |
| `committedTime` | float | `30.0` | Length of the commitment horizon (seconds); used in `ANYTIME` mode (A&ndash;CG) | L<sub>cmt</sub>; starts at 30 s, grows in 5 s steps |
| `NumIter` | int | `1` | Max number of CG iterations per epoch (passive stopping policy &eta;) | A&ndash;CG uses `η = 1` |
| `informTimeLimit` | float | `∞` (`1e7`) | Waiting time after which the passenger is notified of a committed pickup time (seconds) | T<sup>notif</sup>; experiments use `60` |
| `pickupDeviationWindow` | float | `∞` (`1e7`) | Allowable deviation around a committed pickup time (seconds) | &epsilon;; experiments use `60` |
| `nbThreads` | int | `16` | Number of threads for parallel subproblem solving | |

---

## Vehicle Rebalancing / Fleet

| Key | Type | Default | Description | Notes |
|---|---|---|---|---|
| `vehicleReturn` | bool | `0` | Enable idle-vehicle repositioning toward high-demand areas (`1` = enabled) | auxiliary rebalancing |
| `MaxWait` | float | `∞` (`1e7`) | Waiting-time threshold &theta; that marks a committed request as "late" for repositioning (seconds) | &theta;; experiments use `120`&ndash;`360`, calibrated to `300` (5 min) |
| `WaitForReturn` | float | `∞` (`1e7`) | Idle inactivity period before a vehicle becomes eligible for repositioning (seconds) | experiments use `240` |
| `returnType` | int | `1` | Relocation policy: `0` = `TO_SOURCE` (return to origin depot), `1` = `ASSIGN` (return based on assignment) | |

---

## Column Generation

| Key | Type | Default | Description                                                                                                                                | Notes |
|---|---|---|--------------------------------------------------------------------------------------------------------------------------------------------|---|
| `NumColumn` | int | `50` | Max number of columns (routes) added to the RMP per vehicle per CG iteration                                                               | &Omega; |
| `sortColumn` | int | `1` | Column ordering before adding to RMP: `0` = `NORMAL_RC`, `1` = `RC` (reduced cost), `2` = `LAMBDA_S`, `3` = `COMP_C`                       | |
| `Route_Recycle` | bool | `0` | Reuse routes from the previous epoch as a warm start / persistent pool (`1` = enabled)                                                     | "keep" scenarios |
| `reducedCostThreshold` | double | `100.0` | Reduced-cost threshold &gamma; above which pooled columns are pruned (used in `BY_POOL` reuse)                                             | &gamma;; experiments use `150` |
| `InitialDual` | int | `1` | Dual initialization: `0` = `PENALTIES`, `1` = `LAST_LP`, `2` = `BARRIER`, `3` = `INITIAL_LP`, `4` = `GREEDY_D`,                            | `0` = cold start, `1` = dual warm start |
| `DualMethod` | int | `0` | Dual extraction method: `0` = `LMP`, `1` = `INTERIOR`                                                                   | |
| `SmoothDual` | bool | `0` | Apply smoothing to dual variables between CG iterations (`1` = enabled)                                                                    | |
| `warmStart` | int | `1` | Primal warm-start strategy: `0` = `GREEDY_START`, `1` = `PRE_SOLUTION`, `2` = `EMPTY_ROUTES`                                               | |
| `MIPGap` | float | `0.001` | MIP relative optimality gap (`0.001` = 0.1 %)                                                                                              | |
| `solutionMode` | int | `1` | Solve mode: `0` = `STATIC` (offline), `1` = `DYNAMIC` (B&ndash;CG / fixed-epoch batching), `2` = `ANYTIME` (A&ndash;CG / flexible horizon) | |
| `mainAlgorithm` | int | `0` | Algorithm variant: `0` = `GREEDY`, `1` = `MIP`, `2` = `RT_CG` (B&ndash;CG), `3` = `MP_ISUD`, `4` = `MP_MIP`, `5` = `MP_CP`, `6` = `A_CG`   | |

---

## ISUD / MIP Solver

| Key | Type | Default | Description | Notes |
|---|---|---|---|---|
| `isudVariant` | int | `1` | ISUD solve strategy: `0` = `ISUD_MIP_RP`, `1` = `ISUD_PIVOT_RP` | |
| `reducedCP` | bool | `0` | Use a reduced constraint set in the CP step (`1` = enabled) | |
| `minImp` | float | `0.0025` | Minimum improvement threshold for ISUD | |
| `BigM` | int | `27000` | Big-M value used in MIP formulations | key spelled `BigM` |
| `solveTimeLimit` | int | `300` | Time limit for solving MIP models (seconds) | |
| `populateTimeLimit` | int | `200` | Time limit for populating MIP models (seconds) | |

---

## Subproblem / Labeling

| Key | Type | Default | Description                                                                                                                             | Notes |
|---|---|---|-----------------------------------------------------------------------------------------------------------------------------------------|---|
| `subproblemAlgorithm` | int | `1` | Subproblem solver: `0` = `MIP_SUB`, `1` = `LABEL_SETTING`                                                                               | |
| `nbPick` | int | `4` | Maximum number of pickups per route in labeling                                                                                         | M<sup>Pick</sup>; A&ndash;CG fixes `2`, B&ndash;CG uses dynamic up to `4` |
| `Dynamic_Pricing` | bool | `0` | Gradually increase the pickup limit `nbPick` across SP iterations (`1` = enabled)                                                       | mutually exclusive with `Partial_Pricing` |
| `Partial_Pricing` | bool | `0` | Solve the subproblem for a subset of vehicles per iteration (`1` = enabled)                                                             | mutually exclusive with `Dynamic_Pricing` |
| `pruneNodes` | bool | `1` | Remove nodes that cannot improve the best solution (`1` = enabled)                                                                      | Corollary 1 |
| `pruneArcs` | bool | `1` | Remove arcs that cannot improve the best solution (`1` = enabled)                                                                       | Corollary 2 |
| `discardSuboptimalPath` | bool | `1` | Discard partial paths dominated / outside the penalty window (`1` = enabled)                                                            | Corollary 3 |
| `isDropPickPossible` | bool | `0` | Allow visiting a pickup after a drop-off on the same route (`1` = enabled)                                                              | "drop&ndash;pick" restriction is the inverse |
| `isTruncated` | bool | `1` | Truncate the label search when `MaxLabel` is reached (`1` = enabled)                                                                    | |
| `MaxLabel` | int | `15` | Max number of labels retained per node during truncated labeling                                                                        | M<sub>Label</sub> |
| `MaxCommittedLabel` | int | `0` | Max number of committed labels retained per node during truncated labeling                                                              | |
| `sortPath` | int | `0` | Label ordering for truncation: `0` = `L_SCORE` (normalized reduced cost), `1` = `RD_COST` (reduced cost), `2` = `LAMBDA` (lambda score) | `0` recommended (B&ndash;CG) |
| `LabelingStrategy` | int | `1` | Node traversal strategy: `0` = `PUSHING`, `1` = `PULLING`                                                                               | |
| `isDominanceReleased` | bool | `0` | Relax the last condition of the dominance rule to dominate more labels (`1` = enabled)                                                  | "relaxed dominance" |
| `reoptimizeSP` | bool | `0` | Re-optimize subproblems or solve from scratch (`1` = enabled)                                                                           | |
| `LabelingReOptimizeStrategy` | int | `2` | Reduced-network / reuse mode: `0` = `RE_INSERT`, `1` = `BY_BASIS`, `2` = `BY_POOL`                                                      | A&ndash;CG compares `BY_BASIS` vs `BY_POOL` |
| `newRequestLimit` | int | `∞` (`1e7`) | Threshold on the number of new requests that triggers re-optimization                                                                   | |


---

## Validation Rules

The parser rejects invalid combinations:

- `Dynamic_Pricing` and `Partial_Pricing` **cannot** both be enabled in the same scenario.
- A scenario name is required (`--scenario <key>`); the parser throws if it is missing or not
  found under the `scenarios` object.
