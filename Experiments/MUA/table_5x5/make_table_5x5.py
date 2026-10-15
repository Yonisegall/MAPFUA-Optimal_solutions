#!/usr/bin/env python
"""
5x5 MUA experiment: plan, dataset validation, and paper-style summary tables.

This script never generates instances and never runs Java. It only:

    plan      write/validate experiment_plan_5x5.csv (65 configurations, 10 instances each)
    validate  check a user-made instance manifest against the plan (exactly 10 distinct
              random instances per configuration; reports missing/extra/duplicated ones)
    table     consume run_mua_campaign.py results and write the summary tables
    preview   print an empty table (structure check only)

Pipeline:
    (user creates 5x5 instances + manifest.csv)
    python make_table_5x5.py validate --manifest <manifest.csv>
    python ../run_mua_campaign.py --manifest <manifest.csv> --results table_5x5/results --timeout-ms 120000
    python make_table_5x5.py table --runs results/runner/campaign_runs.csv --manifest <manifest.csv>

------------------------------------------------------------------------------------------
Displayed metric -> raw result field
------------------------------------------------------------------------------------------
Every run writes one Java result CSV ("res run <timestamp> .csv", path stored in the
runner's campaign_runs.csv column `result_csv`). Fields are InstanceReport.StandardFields:

  Runtime (all solvers)  <- "Elapsed Time (ms)"            (A_Solver: solver wall time,
                                                            excludes JVM start-up; shown in s)
  A*+OD Nodes            <- "Expanded Nodes (High Level)"   (AStarODMuaSolver.expandedNodes:
                                                            every expanded OD node, partial and
                                                            full; identical to MAPFUA_MA_main_expanded,
                                                            which is used as a fallback only)
  CBS* HL Nodes          <- "Expanded Nodes (High Level)"   (CBSStarMUA_Solver.expandedNodes:
                                                            expanded CBS* constraint-tree nodes)
  CBS* LL Nodes          <- "Expanded Nodes (Low Level)"    (sum over all low-level calls of
                                                            CBSStarJointLowLevel.expanded())

Run status comes from the runner's `status` column (SOLVED / TIMEOUT / OOM / UNSOLVED /
ERROR). Only SOLVED runs (Solved=1 and Valid=1) ever contribute a runtime or node count;
a timeout is never treated as a solved runtime.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Optional

HERE = Path(__file__).resolve().parent
PLAN_PATH = HERE / "experiment_plan_5x5.csv"

GRID_SIZE = "5x5"
INSTANCES_PER_CONFIG = 10

# Exactly the configurations requested for the experiment (num_aa -> total_agents values).
CONFIGURATIONS = {
    2: [3, 5, 7, 9, 11, 13, 15, 17, 19],
    3: [4, 6, 8, 10, 12, 14, 16, 18, 20],
    4: [5, 7, 9, 11, 13, 15, 17, 19],
    5: [6, 8, 10, 12, 14, 16, 18, 20],
    6: [7, 9, 11, 13, 15, 17, 19],
    7: [8, 10, 12, 14, 16, 18, 20],
    8: [9, 11, 13, 15, 17, 19],
    9: [10, 12, 14, 16, 18, 20],
    10: [11, 13, 15, 17, 19],
}
EXPECTED_CONFIGURATIONS = 65
EXPECTED_INSTANCES = EXPECTED_CONFIGURATIONS * INSTANCES_PER_CONFIG

# Table column groups, in display order: (solver, display name, [(metric key, header)]).
SOLVER_GROUPS = [
    ("AStarODMUAZero", "A*+OD Zero", [("runtime", "Runtime"), ("nodes", "Nodes")]),
    ("AStarODMUAMax", "A*+OD MAX", [("runtime", "Runtime"), ("nodes", "Nodes")]),
    ("AStarODMUAIH", "A*+OD IH", [("runtime", "Runtime"), ("nodes", "Nodes")]),
    ("AStarODMUAMH", "A*+OD MH", [("runtime", "Runtime"), ("nodes", "Nodes")]),
    ("CBSStarMUA", "CBS*", [("runtime", "Runtime"), ("ll_nodes", "LL Nodes"), ("hl_nodes", "HL Nodes")]),
]
CANONICAL_SOLVERS = [s for s, _, _ in SOLVER_GROUPS]
ASTAR_SOLVERS = [s for s in CANONICAL_SOLVERS if s.startswith("AStarOD")]

# Raw result-CSV field per metric, per solver family (see module docstring).
RUNTIME_FIELD = "Elapsed Time (ms)"
HL_EXPANDED_FIELD = "Expanded Nodes (High Level)"
LL_EXPANDED_FIELD = "Expanded Nodes (Low Level)"
ASTAR_EXPANDED_FALLBACK = "MAPFUA_MA_main_expanded"

AGGREGATIONS = {"mean": statistics.fmean, "median": statistics.median}
TERMINAL_STATUSES = {"SOLVED", "TIMEOUT", "OOM", "UNSOLVED", "ERROR"}

# Cell markers (never NaN in a presented table).
MARK_TIMEOUT = "TO"     # runs exist, none solved within the timeout (timeout/OOM/unsolved)
MARK_ERROR = "ERR"      # runs exist, all of them ERROR
MARK_NO_DATA = "--"     # no run at all for this (solver, configuration)


# ======================================================================================
# Plan
# ======================================================================================

def plan_rows() -> list[dict]:
    rows = []
    for num_aa, totals in CONFIGURATIONS.items():
        for total in totals:
            rows.append({
                "grid_size": GRID_SIZE,
                "num_aa": num_aa,
                "total_agents": total,
                "num_ua": total - num_aa,
                "expected_random_instances": INSTANCES_PER_CONFIG,
            })
    return rows


def validate_plan_rows(rows: list[dict]) -> list[str]:
    errors = []
    keys = [(int(r["num_aa"]), int(r["total_agents"])) for r in rows]
    if len(keys) != EXPECTED_CONFIGURATIONS:
        errors.append(f"expected {EXPECTED_CONFIGURATIONS} configurations, found {len(keys)}")
    if len(set(keys)) != len(keys):
        errors.append("duplicated configurations in plan")
    expected_keys = {(a, t) for a, ts in CONFIGURATIONS.items() for t in ts}
    if set(keys) != expected_keys:
        errors.append(f"plan differs from the required configurations: "
                      f"missing={sorted(expected_keys - set(keys))} extra={sorted(set(keys) - expected_keys)}")
    for r in rows:
        if r["grid_size"] != GRID_SIZE:
            errors.append(f"bad grid_size {r['grid_size']!r}")
        if int(r["num_ua"]) != int(r["total_agents"]) - int(r["num_aa"]) or int(r["num_ua"]) < 1:
            errors.append(f"bad num_ua in {r}")
        if int(r["expected_random_instances"]) != INSTANCES_PER_CONFIG:
            errors.append(f"expected_random_instances != {INSTANCES_PER_CONFIG} in {r}")
    total = sum(int(r["expected_random_instances"]) for r in rows)
    if total != EXPECTED_INSTANCES:
        errors.append(f"expected {EXPECTED_INSTANCES} instances in total, plan implies {total}")
    return errors


def load_plan() -> list[dict]:
    if not PLAN_PATH.is_file():
        raise SystemExit(f"{PLAN_PATH} missing; run: python make_table_5x5.py plan")
    rows = read_csv(PLAN_PATH)
    errors = validate_plan_rows(rows)
    if errors:
        raise SystemExit("Invalid plan:\n  " + "\n  ".join(errors))
    for r in rows:
        for k in ("num_aa", "total_agents", "num_ua", "expected_random_instances"):
            r[k] = int(r[k])
    return rows


def cmd_plan(args) -> int:
    rows = plan_rows()
    errors = validate_plan_rows(rows)
    if errors:
        print("Generated plan is invalid:\n  " + "\n  ".join(errors))
        return 1
    write_csv(PLAN_PATH, list(rows[0].keys()), rows)
    print(f"Wrote {PLAN_PATH}")
    print(f"configurations = {len(rows)} (expected {EXPECTED_CONFIGURATIONS})")
    print(f"eventual instances = {sum(r['expected_random_instances'] for r in rows)} (expected {EXPECTED_INSTANCES})")
    return 0


# ======================================================================================
# Manifest validation
# ======================================================================================

def manifest_grid_size(row: dict) -> str:
    g = (row.get("grid_size") or "").strip()
    if g:
        return g
    w, h = row.get("grid_width"), row.get("grid_height")
    return f"{int(float(w))}x{int(float(h))}" if w and h else ""


def manifest_config(row: dict) -> tuple[Optional[int], Optional[int]]:
    aa, total = as_int(row.get("num_aa")), as_int(row.get("total_agents"))
    return aa, total


def validate_dataset(instance_rows: list[dict], plan: list[dict], source: str) -> tuple[bool, list[str]]:
    """
    instance_rows: one row per instance (manifest rows, or distinct instances seen in results).
    Returns (complete, report_lines). Complete means: every planned configuration has exactly
    10 distinct instances, nothing outside the plan, no duplicated instance names/scenarios,
    and grid_size/num_ua consistent.
    """
    lines: list[str] = []
    problems = 0
    planned = {(r["num_aa"], r["total_agents"]): r for r in plan}
    by_config: dict[tuple, list[dict]] = {}
    seen_names: dict[str, int] = {}
    seen_scen: dict[str, int] = {}

    for r in instance_rows:
        name = (r.get("instance_name") or "").strip()
        seen_names[name] = seen_names.get(name, 0) + 1
        scen = (r.get("scenario_path") or "").strip()
        if scen:
            seen_scen[scen] = seen_scen.get(scen, 0) + 1
        aa, total = manifest_config(r)
        grid = manifest_grid_size(r)
        if grid and grid != GRID_SIZE:
            lines.append(f"  WRONG GRID      {name}: grid_size={grid!r} (expected {GRID_SIZE})")
            problems += 1
        ua = as_int(r.get("num_ua"))
        if aa is not None and total is not None and ua is not None and ua != total - aa:
            lines.append(f"  WRONG NUM_UA    {name}: num_ua={ua} but total_agents-num_aa={total - aa}")
            problems += 1
        by_config.setdefault((aa, total), []).append(r)

    for name, n in seen_names.items():
        if n > 1:
            lines.append(f"  DUPLICATE NAME  {name!r} appears {n} times")
            problems += 1
    for scen, n in seen_scen.items():
        if n > 1:
            lines.append(f"  DUPLICATE SCEN  {scen!r} appears {n} times")
            problems += 1

    for key in sorted(k for k in by_config if k not in planned):
        lines.append(f"  NOT IN PLAN     num_aa={key[0]} total_agents={key[1]}: "
                     f"{len(by_config[key])} instance(s), e.g. {by_config[key][0].get('instance_name')}")
        problems += 1

    for key, p in planned.items():
        rows = by_config.get(key, [])
        distinct = {(r.get("instance_name") or "").strip() for r in rows}
        n = len(distinct)
        label = f"{key[0]}/{key[1]}"
        if n == 0:
            lines.append(f"  MISSING CONFIG  {label}: 0/{p['expected_random_instances']} instances")
            problems += 1
        elif n != p["expected_random_instances"]:
            kind = "TOO FEW" if n < p["expected_random_instances"] else "TOO MANY"
            lines.append(f"  {kind:<15} {label}: {n}/{p['expected_random_instances']} distinct instances")
            problems += 1

    n_instances = len({(r.get("instance_name") or "").strip() for r in instance_rows})
    n_cfg = sum(1 for k in planned if by_config.get(k))
    header = [f"Dataset check ({source}): {n_instances} distinct instances, "
              f"{n_cfg}/{len(planned)} planned configurations present, "
              f"expected {EXPECTED_INSTANCES} instances."]
    complete = problems == 0
    header.append("  COMPLETE: exactly 10 distinct instances for every configuration."
                  if complete else f"  INCOMPLETE: {problems} problem(s):")
    return complete, header + lines


def cmd_validate(args) -> int:
    plan = load_plan()
    rows = read_csv(Path(args.manifest))
    missing_cols = [c for c in ("instance_name", "scenario_path", "num_aa", "total_agents", "num_ua")
                    if rows and c not in rows[0]]
    if missing_cols:
        print(f"Manifest is missing required column(s): {missing_cols}")
        return 1
    complete, report = validate_dataset(rows, plan, f"manifest {args.manifest}")
    print("\n".join(report))
    return 0 if complete else 1


# ======================================================================================
# Results -> per-run records
# ======================================================================================

def resolve_result_csv(rel: str, repo_root: Path, results_root: Path) -> Optional[Path]:
    if not rel:
        return None
    p = Path(rel)
    candidates = [p] if p.is_absolute() else [repo_root / p]
    parts = list(Path(rel.replace("\\", "/")).parts)
    if "raw" in parts:  # results directory copied elsewhere (e.g. back from the cluster)
        candidates.append(results_root.joinpath(*parts[parts.index("raw"):]))
    for c in candidates:
        if c.is_file():
            return c
    return None


def load_run_records(runs_csv: Path, timeout_ms: Optional[int], manifest_by_name: Optional[dict]) -> tuple[list[dict], list[str]]:
    repo_root = find_repo_root(HERE)
    results_root = runs_csv.resolve().parent.parent
    warnings: list[str] = []
    rows = read_csv(runs_csv)
    if timeout_ms is not None:
        rows = [r for r in rows if as_int(r.get("timeout_ms")) == timeout_ms]
    rows = [r for r in rows if r.get("solver") in CANONICAL_SOLVERS]

    timeouts = sorted({r.get("timeout_ms") for r in rows})
    if len(timeouts) > 1:
        raise SystemExit(f"campaign_runs.csv mixes timeouts {timeouts}; pass --timeout-ms to choose one.")

    seen: dict[tuple, int] = {}
    records = []
    for r in rows:
        name = r.get("instance_name", "")
        key = (name, r["solver"])
        seen[key] = seen.get(key, 0) + 1
        meta = manifest_by_name.get(name) if manifest_by_name is not None else r
        if meta is None:
            warnings.append(f"run for {name!r} ({r['solver']}) is not in the manifest; ignored")
            continue
        status = r.get("status", "")
        rec = {
            "instance_name": name,
            "solver": r["solver"],
            "grid_size": manifest_grid_size(meta) or r.get("grid_size", ""),
            "num_aa": as_int(meta.get("num_aa")),
            "total_agents": as_int(meta.get("total_agents")),
            "num_ua": as_int(meta.get("num_ua")),
            "status": status if status in TERMINAL_STATUSES else "INCOMPLETE",
            "timeout_ms": as_int(r.get("timeout_ms")),
            "runtime_s": None, "nodes": None, "hl_nodes": None, "ll_nodes": None,
        }
        if rec["status"] == "SOLVED":
            raw = None
            path = resolve_result_csv(r.get("result_csv", ""), repo_root, results_root)
            if path is not None:
                raw_rows = read_csv(path)
                raw = raw_rows[0] if len(raw_rows) == 1 else None
            if raw is None:
                warnings.append(f"SOLVED run without readable result CSV: {name} / {r['solver']} ({r.get('result_csv')})")
                runtime_ms = as_float(r.get("solver_elapsed_ms"))
            else:
                runtime_ms = as_float(raw.get(RUNTIME_FIELD))
                hl = as_float(raw.get(HL_EXPANDED_FIELD))
                if r["solver"] in ASTAR_SOLVERS:
                    rec["nodes"] = hl if hl is not None else as_float(raw.get(ASTAR_EXPANDED_FALLBACK))
                else:
                    rec["hl_nodes"] = hl
                    rec["ll_nodes"] = as_float(raw.get(LL_EXPANDED_FIELD))
            rec["runtime_s"] = runtime_ms / 1000.0 if runtime_ms is not None else None
        records.append(rec)

    dups = {k: n for k, n in seen.items() if n > 1}
    if dups:
        raise SystemExit("Duplicated (instance, solver) runs in campaign_runs.csv: "
                         + ", ".join(f"{k[0]}/{k[1]} x{n}" for k, n in sorted(dups.items())))
    return records, warnings


# ======================================================================================
# Aggregation
# ======================================================================================

def metric_keys(solver: str) -> list[str]:
    return [m for s, _, ms in SOLVER_GROUPS if s == solver for m, _ in ms]


def coverage_rows(records: list[dict], plan: list[dict]) -> list[dict]:
    out = []
    for p in plan:
        for solver in CANONICAL_SOLVERS:
            rs = [r for r in records if r["solver"] == solver
                  and r["num_aa"] == p["num_aa"] and r["total_agents"] == p["total_agents"]]
            count = {s: sum(1 for r in rs if r["status"] == s) for s in TERMINAL_STATUSES}
            completed = sum(count.values())
            expected = p["expected_random_instances"]
            missing_metrics = sum(1 for r in rs if r["status"] == "SOLVED"
                                  and any(r[m if m != "runtime" else "runtime_s"] is None for m in metric_keys(solver)))
            out.append({
                "solver": solver, "num_aa": p["num_aa"], "total_agents": p["total_agents"],
                "num_ua": p["num_ua"], "expected_instances": expected,
                "runs_found": len(rs), "completed": completed,
                "solved": count["SOLVED"], "timeout": count["TIMEOUT"], "oom": count["OOM"],
                "unsolved": count["UNSOLVED"], "error": count["ERROR"],
                "missing": max(expected - completed, 0),
                "solved_missing_metrics": missing_metrics,
                "success_rate": round(count["SOLVED"] / expected, 4),
            })
    return out


def aggregate_cell(rs: list[dict], metric: str, agg: str, policy: str, expected: int) -> tuple[Optional[float], str]:
    """Returns (value or None, marker/status). Status: OK, PARTIAL, TO, ERR, NO_DATA."""
    field = "runtime_s" if metric == "runtime" else metric
    completed = [r for r in rs if r["status"] in TERMINAL_STATUSES]
    solved = [r for r in completed if r["status"] == "SOLVED"]
    if not completed:
        return None, "NO_DATA"
    if not solved:
        return None, "ERR" if all(r["status"] == "ERROR" for r in completed) else "TO"
    if policy == "all-solved" and len(solved) < expected:
        return None, "TO"
    values = [r[field] for r in solved if r[field] is not None]
    if not values:
        return None, "NO_DATA"
    return AGGREGATIONS[agg](values), ("OK" if len(solved) >= expected else "PARTIAL")


def summary_rows(records: list[dict], plan: list[dict], agg: str, policy: str) -> list[dict]:
    out = []
    for p in plan:
        row = {"agents": f"{p['num_aa']}/{p['total_agents']}", "grid_size": p["grid_size"],
               "num_aa": p["num_aa"], "total_agents": p["total_agents"], "num_ua": p["num_ua"],
               "expected_instances": p["expected_random_instances"], "aggregation": agg, "policy": policy}
        for solver in CANONICAL_SOLVERS:
            rs = [r for r in records if r["solver"] == solver
                  and r["num_aa"] == p["num_aa"] and r["total_agents"] == p["total_agents"]]
            row[f"{solver}_solved"] = sum(1 for r in rs if r["status"] == "SOLVED")
            for metric in metric_keys(solver):
                value, status = aggregate_cell(rs, metric, agg, policy, p["expected_random_instances"])
                row[f"{solver}_{metric}"] = "" if value is None else round(value, 6)
                row[f"{solver}_{metric}_status"] = status
        out.append(row)
    return out


# ======================================================================================
# Formatting
# ======================================================================================

def fmt_runtime(v: float) -> str:
    return f"{v:.2f}" if v < 10 else f"{v:.1f}"


def fmt_nodes(v: float, latex: bool) -> str:
    s = f"{v:,.0f}"
    return s.replace(",", "{,}") if latex else s


def display_cell(row: dict, solver: str, metric: str, latex: bool, bold: bool) -> str:
    status = row[f"{solver}_{metric}_status"]
    if status in ("TO", "ERR", "NO_DATA"):
        return {"TO": MARK_TIMEOUT, "ERR": MARK_ERROR, "NO_DATA": MARK_NO_DATA}[status]
    v = float(row[f"{solver}_{metric}"])
    s = fmt_runtime(v) if metric == "runtime" else fmt_nodes(v, latex)
    if bold:
        s = f"\\textbf{{{s}}}" if latex else f"**{s}**"
    if status == "PARTIAL":
        n = row[f"{solver}_solved"]
        s = f"{s}$^{{{n}}}$" if latex else f"{s} ({n}/{row['expected_instances']})"
    return s


def best_cells(row: dict) -> set[tuple[str, str]]:
    """Bold: lowest runtime over all solvers, lowest expanded nodes among A*+OD heuristics.
    Only fully-covered (OK) cells compete, so a partial average can never win."""
    best = set()
    for metric, solvers in (("runtime", CANONICAL_SOLVERS), ("nodes", ASTAR_SOLVERS)):
        vals = {s: float(row[f"{s}_{metric}"]) for s in solvers if row[f"{s}_{metric}_status"] == "OK"}
        if len(vals) >= 2:
            m = min(vals.values())
            best |= {(s, metric) for s, v in vals.items() if v == m}
    return best


def latex_table(rows: list[dict], caption: str, label: str, bold: bool) -> str:
    n_cols = 1 + sum(len(ms) for _, _, ms in SOLVER_GROUPS)
    spec = "l" + "".join("|" + "r" * len(ms) for _, _, ms in SOLVER_GROUPS)
    top = ["\\multirow{2}{*}{Agents}"]
    rules, sub = [], [""]
    col = 2
    for _, display, ms in SOLVER_GROUPS:
        top.append(f"\\multicolumn{{{len(ms)}}}{{c}}{{{display}}}")
        rules.append(f"\\cmidrule(lr){{{col}-{col + len(ms) - 1}}}")
        sub += [h for _, h in ms]
        col += len(ms)
    lines = [
        "% Generated by make_table_5x5.py. Requires \\usepackage{booktabs,multirow}.",
        "\\begin{table*}[t]", "\\centering", "\\small",
        f"\\caption{{{caption}}}", f"\\label{{{label}}}",
        f"\\begin{{tabular}}{{{spec}}}", "\\toprule",
        " & ".join(top) + " \\\\", "".join(rules),
        " & ".join(sub) + " \\\\", "\\midrule",
    ]
    prev_aa = None
    for r in rows:
        if prev_aa is not None and r["num_aa"] != prev_aa:
            lines.append("\\midrule")
        prev_aa = r["num_aa"]
        best = best_cells(r) if bold else set()
        cells = [r["agents"]] + [display_cell(r, s, m, True, (s, m) in best)
                                 for s, _, ms in SOLVER_GROUPS for m, _ in ms]
        assert len(cells) == n_cols
        lines.append(" & ".join(cells) + " \\\\")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table*}", ""]
    return "\n".join(lines)


def caption_for(agg: str, policy: str, timeout_s: Optional[float], aa: Optional[int] = None) -> str:
    who = f"{aa} assigned agents" if aa is not None else "all configurations"
    rule = ("over solved instances only; $^{k}$ = only $k$ of 10 solved"
            if policy == "solved-only" else "reported only when all 10 instances are solved")
    to = f"{timeout_s:g}\\,s" if timeout_s else "the time limit"
    return (f"5x5 grid, {who}. Agents = AA/total agents (10 random instances each). "
            f"{agg.capitalize()} runtime (s) and expanded nodes, {rule}. "
            f"{MARK_TIMEOUT} = no instance solved within {to}; {MARK_NO_DATA} = no runs.")


def markdown_table(rows: list[dict], bold: bool) -> str:
    head = ["Agents"] + [f"{d} {h}" for _, d, ms in SOLVER_GROUPS for _, h in ms]
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for r in rows:
        best = best_cells(r) if bold else set()
        cells = [r["agents"]] + [display_cell(r, s, m, False, (s, m) in best)
                                 for s, _, ms in SOLVER_GROUPS for m, _ in ms]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines) + "\n"


# ======================================================================================
# Commands
# ======================================================================================

def cmd_table(args) -> int:
    plan = load_plan()
    manifest_by_name = None
    if args.manifest:
        mrows = read_csv(Path(args.manifest))
        manifest_by_name = {}
        for r in mrows:
            manifest_by_name.setdefault(r.get("instance_name", ""), r)
        complete, report = validate_dataset(mrows, plan, f"manifest {args.manifest}")
    records, warnings = load_run_records(Path(args.runs), args.timeout_ms, manifest_by_name)
    if not args.manifest:
        distinct = {}
        for r in records:
            distinct.setdefault(r["instance_name"], r)
        complete, report = validate_dataset(list(distinct.values()), plan, f"results {args.runs}")
    print("\n".join(report))
    for w in warnings:
        print("WARNING:", w)
    if not complete and args.require_complete:
        print("Refusing to write tables: dataset incomplete (--require-complete).")
        return 1

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    timeout_s = (records[0]["timeout_ms"] / 1000.0) if records and records[0]["timeout_ms"] else None

    cov = coverage_rows(records, plan)
    write_csv(out / "coverage_5x5.csv", list(cov[0].keys()), cov)
    incomplete_cells = [c for c in cov if c["completed"] < c["expected_instances"]]
    partial_cells = [c for c in cov if 0 < c["solved"] < c["expected_instances"]]
    print(f"coverage: {len(cov)} solver x configuration cells; "
          f"{len(incomplete_cells)} with missing runs, {len(partial_cells)} partially solved")

    written = [out / "coverage_5x5.csv"]
    summaries = {}
    for agg in args.agg:
        rows = summary_rows(records, plan, agg, args.policy)
        summaries[agg] = rows
        write_csv(out / f"table_5x5_{agg}.csv", list(rows[0].keys()), rows)
        (out / f"table_5x5_{agg}.tex").write_text(
            latex_table(rows, caption_for(agg, args.policy, timeout_s), f"tab:mua5x5-{agg}", not args.no_bold),
            encoding="utf-8")
        (out / f"table_5x5_{agg}.md").write_text(
            f"# 5x5 MUA table ({agg}, policy={args.policy})\n\n" + markdown_table(rows, not args.no_bold),
            encoding="utf-8")
        written += [out / f"table_5x5_{agg}.{ext}" for ext in ("csv", "tex", "md")]

    per_aa_agg = args.per_aa_agg if args.per_aa_agg in summaries else args.agg[0]
    for aa in CONFIGURATIONS:
        rows = [r for r in summaries[per_aa_agg] if r["num_aa"] == aa]
        p = out / f"table_5x5_aa{aa}.tex"
        p.write_text(latex_table(rows, caption_for(per_aa_agg, args.policy, timeout_s, aa),
                                 f"tab:mua5x5-aa{aa}", not args.no_bold), encoding="utf-8")
        written.append(p)
    print(f"per-AA tables use aggregation: {per_aa_agg}")
    print("Wrote:\n  " + "\n  ".join(str(p) for p in written))
    return 0


def cmd_preview(args) -> int:
    plan = load_plan()
    rows = summary_rows([], plan[: args.rows] if args.rows else plan, "mean", "solved-only")
    print(markdown_table(rows, False))
    if args.latex:
        print(latex_table(rows, caption_for("mean", "solved-only", None), "tab:mua5x5-preview", False))
    return 0


# ======================================================================================
# Small utilities
# ======================================================================================

def read_csv(path: Path) -> list[dict]:
    with open(path, "r", encoding="utf-8-sig", newline="") as f:
        return [dict(r) for r in csv.DictReader(f)]


def write_csv(path: Path, fields: list[str], rows: list[dict]):
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def as_float(v) -> Optional[float]:
    try:
        x = float(str(v).strip().strip('"'))
        return None if math.isnan(x) else x
    except (TypeError, ValueError):
        return None


def as_int(v) -> Optional[int]:
    x = as_float(v)
    return int(x) if x is not None else None


def find_repo_root(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "pom.xml").is_file():
            return p
    return start


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("plan", help="write and validate experiment_plan_5x5.csv")

    v = sub.add_parser("validate", help="check an instance manifest against the plan")
    v.add_argument("--manifest", required=True)

    t = sub.add_parser("table", help="build tables from run_mua_campaign.py results")
    t.add_argument("--runs", required=True, help="results/runner/campaign_runs.csv")
    t.add_argument("--manifest", default=None,
                   help="instance manifest (authoritative num_aa/total_agents; enables completeness check)")
    t.add_argument("--out", default=str(HERE / "output"))
    t.add_argument("--agg", nargs="+", choices=list(AGGREGATIONS), default=["mean", "median"])
    t.add_argument("--per-aa-agg", choices=list(AGGREGATIONS), default="mean",
                   help="aggregation used for the per-AA tables table_5x5_aa<N>.tex")
    t.add_argument("--policy", choices=["solved-only", "all-solved"], default="solved-only",
                   help="solved-only: aggregate the solved instances, mark partial cells with the "
                        "solved count; all-solved: show a value only if all 10 are solved, else TO")
    t.add_argument("--timeout-ms", type=int, default=None, help="select runs with this timeout")
    t.add_argument("--require-complete", action="store_true",
                   help="fail instead of writing tables when the dataset is incomplete")
    t.add_argument("--no-bold", action="store_true", help="do not bold the best cells")

    pv = sub.add_parser("preview", help="print an empty table to check the structure")
    pv.add_argument("--rows", type=int, default=0, help="only the first N configurations")
    pv.add_argument("--latex", action="store_true")

    args = p.parse_args(argv)
    return {"plan": cmd_plan, "validate": cmd_validate, "table": cmd_table, "preview": cmd_preview}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
