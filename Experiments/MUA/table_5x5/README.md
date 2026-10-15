# 5x5 MUA experiment: table infrastructure

A self-contained experiment comparing CBS* with A*+OD (Zero, MAX, IH, MH) on random 5x5
grids. This folder holds the plan, a dataset validator, and the table builder. It does
**not** generate instances and does **not** run solvers.

```
table_5x5/
  experiment_plan_5x5.csv   65 configurations x 10 random instances = 650 instances
  make_table_5x5.py         plan / validate / table / preview
  results/                  run_mua_campaign.py --results target (raw/ + runner/)
  output/                   generated tables
```

## Configurations

Grid 5x5. Every agent that is not an AA is a UA (`num_ua = total_agents - num_aa`).

| num_aa | total_agents |
|---|---|
| 2 | 3, 5, ..., 19 |
| 3 | 4, 6, ..., 20 |
| 4 | 5, 7, ..., 19 |
| 5 | 6, 8, ..., 20 |
| 6 | 7, 9, ..., 19 |
| 7 | 8, 10, ..., 20 |
| 8 | 9, 11, ..., 19 |
| 9 | 10, 12, ..., 20 |
| 10 | 11, 13, ..., 19 |

`python make_table_5x5.py plan` rewrites and validates the plan (65 rows, 650 instances).

## 1. Instances and manifest (created by you)

You create the instances yourself. The runner needs a `manifest.csv` with one row per instance:

| column | meaning |
|---|---|
| `instance_name` | exact `.scen` file name (the runner selects it with `-iRegex ^name$`) |
| `scenario_path` | path to the `.scen`, relative to the repository root |
| `map_name` | the `.map` file, which must sit in the same directory as the `.scen` (`-iDir` is not recursive) |
| `grid_size` | `5x5` |
| `num_aa`, `total_agents`, `num_ua` | configuration; the runner passes `-a total_agents` |
| `seed` | random seed used to make the instance |

In a scenario, a UA is an agent whose target equals its start.

Check the dataset before running anything:

```
python make_table_5x5.py validate --manifest <manifest.csv>
```

It reports configurations that are missing, have too few or too many instances, or are not
in the plan, plus duplicated instance names or scenario paths, wrong grid sizes and wrong
`num_ua`. It exits 0 only when every configuration has exactly 10 distinct instances.

## 2. Runs (cluster)

```
python ../run_mua_campaign.py --manifest <manifest.csv> --results Experiments/MUA/table_5x5/results --timeout-ms 120000
```

The runner launches the solvers `CBSStarMUA, AStarODMUAZero, AStarODMUAMax, AStarODMUAIH,
AStarODMUAMH`, one Java process per (instance, solver), one after another. Use `--dry-run`
first to preview.

## 3. Tables

```
python make_table_5x5.py table --runs results/runner/campaign_runs.csv --manifest <manifest.csv>
```

Writes to `output/`:

| file | content |
|---|---|
| `coverage_5x5.csv` | per solver x (num_aa, total_agents): expected, completed, solved, timeout, oom, unsolved, error, missing, success_rate |
| `table_5x5_mean.{csv,tex,md}` | all 65 rows, mean |
| `table_5x5_median.{csv,tex,md}` | all 65 rows, median |
| `table_5x5_aa2.tex` ... `table_5x5_aa10.tex` | one table per AA count (`--per-aa-agg`, default mean) |

Table layout: `Agents` (AA/total) | A*+OD Zero (Runtime, Nodes) | A*+OD MAX (Runtime, Nodes) |
A*+OD IH (Runtime, Nodes) | A*+OD MH (Runtime, Nodes) | CBS* (Runtime, LL Nodes, HL Nodes).
LaTeX needs `\usepackage{booktabs,multirow}`.

Options:

- `--policy solved-only` (default): aggregate only the solved instances. A cell where only
  k of 10 were solved shows a superscript `k` (LaTeX) or `(k/10)` (Markdown).
- `--policy all-solved`: show a value only when all 10 are solved, otherwise `TO`.
- `--agg mean median` (default: both), `--timeout-ms` (choose one timeout if the runs
  file mixes several), `--require-complete` (refuse to write tables for an incomplete
  dataset), `--no-bold`.

Bold marks the lowest runtime over all solvers and the lowest node count among the A*+OD
variants. Only fully solved cells (10/10) can be bold.

Cell markers: `TO` = runs exist but none were solved (timeout, OOM or unsolved). `ERR` = all
runs errored. `--` = no runs. Timed-out runs never contribute a runtime. The flat CSVs leave
undefined values empty and add a `<solver>_<metric>_status` column (OK / PARTIAL / TO / ERR / NO_DATA).

## Metric -> raw result field

Each run's Java result CSV path is in the `result_csv` column of `campaign_runs.csv`.

| displayed | raw field | source |
|---|---|---|
| Runtime (s), all solvers | `Elapsed Time (ms)` | `A_Solver`, solver time only (no JVM start-up) |
| A*+OD Nodes | `Expanded Nodes (High Level)` | `AStarODMuaSolver.expandedNodes`: every expanded OD node, partial and full; equals `MAPFUA_MA_main_expanded` |
| CBS* HL Nodes | `Expanded Nodes (High Level)` | expanded CBS* constraint-tree nodes |
| CBS* LL Nodes | `Expanded Nodes (Low Level)` | sum of `CBSStarJointLowLevel.expanded()` over all low-level calls |

Run status (SOLVED / TIMEOUT / OOM / UNSOLVED / ERROR) comes from the runner's `status`
column. SOLVED means `Solved=1` and `Valid=1`.

`python make_table_5x5.py preview [--rows N] [--latex]` prints the empty table structure.
