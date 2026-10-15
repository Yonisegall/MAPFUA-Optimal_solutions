#!/usr/bin/env python
"""
Permanent MUA campaign runner.

Pipeline position:
    create_mua_instances.py  -> generate instances
    run_mua_campaign.py      -> execute solver runs robustly   (this file)
    analyze_mua_results.py   -> analyze results

Launches the project's real Java entry point (Main, via GenericRunManager) once
per (instance, solver) pair, each in its own subprocess, so that a crash/OOM/hang
in one run cannot affect any other run. See --help for all options, and run with
--dry-run first to preview a campaign without launching any Java process.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

# --------------------------------------------------------------------------------------
# Constants
# --------------------------------------------------------------------------------------

# Canonical solver order. Also used as the rotation base order (section 13).
CANONICAL_SOLVERS = [
    "CBSStarMUA",
    "AStarODMUAZero",
    "AStarODMUAMax",
    "AStarODMUAIH",
    "AStarODMUAMH",
]

TERMINAL_STATUSES = {"SOLVED", "TIMEOUT", "OOM", "UNSOLVED", "ERROR"}

# Fields that define a "run key": two rows are the same experimental run only if
# ALL of these match. Anything that changes the experimental budget (solver
# timeout, heap, or the instances being solved, via the manifest hash) makes it
# a *different* run, even for the same (instance, solver) pair.
RUN_KEY_FIELDS = ["instance_name", "solver", "timeout_ms", "java_heap", "manifest_sha256"]

RUN_COLUMNS = [
    "campaign_id", "run_id", "run_key", "plan_position", "run_order_index",
    "instance_name", "scenario_path", "map_name", "solver",
    "seed",
    "grid_size", "requested_density", "actual_density",
    "total_agents", "num_aa", "num_ua",
    "requested_ua_fraction", "actual_ua_fraction",
    "timeout_ms", "grace_ms", "java_heap", "manifest_sha256",
    "status",
    "process_return_code", "process_wall_time_ms",
    "result_csv", "log_csv", "stdout_path", "stderr_path",
    "solved", "valid", "timed_out", "mua", "solver_elapsed_ms",
    "error_type", "error_message",
    "started_at", "finished_at",
]

SUMMARY_COLUMNS = [
    "solver", "expected", "completed", "solved", "timeout", "oom", "unsolved",
    "error", "success_rate",
]

OOM_PATTERNS = (
    "OutOfMemoryError",
    "Java heap space",
    "GC overhead limit exceeded",
)


# --------------------------------------------------------------------------------------
# Small utilities
# --------------------------------------------------------------------------------------

def eprint(*a, **kw):
    print(*a, file=sys.stderr, **kw)


def find_repo_root(start: Path) -> Path:
    cur = start.resolve()
    for candidate in [cur, *cur.parents]:
        if (candidate / "pom.xml").is_file():
            return candidate
    raise RuntimeError(f"Could not locate repository root (pom.xml) above {start}")


def sha256_of_file(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def col(row: dict, *names, default=None):
    """Tolerant manifest column lookup: first present & non-empty value among candidate names."""
    for n in names:
        v = row.get(n)
        if v is not None and str(v).strip() != "":
            return v
    return default


def as_int(v, default=None):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default


def as_float(v, default=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def atomic_write_text(path: Path, text: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + f".tmp{os.getpid()}")
    with open(tmp, "w", encoding="utf-8", newline="") as f:
        f.write(text)
    os.replace(tmp, path)


def atomic_write_csv(path: Path, fieldnames: list[str], rows: list[dict]):
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=fieldnames, extrasaction="ignore")
    writer.writeheader()
    for r in rows:
        writer.writerow({k: ("" if r.get(k) is None else r.get(k)) for k in fieldnames})
    atomic_write_text(path, buf.getvalue())


# --------------------------------------------------------------------------------------
# Java launch mechanism discovery
# --------------------------------------------------------------------------------------

def discover_java_home(repo_root: Path) -> Optional[str]:
    """
    Discover the project's configured JDK, mirroring how VS Code launches this
    project (.vscode/settings.json -> java.configuration.runtimes, the entry
    marked "default": true). Falls back to JAVA_HOME, then None (caller falls
    back further to 'java' on PATH).
    """
    settings_path = repo_root / ".vscode" / "settings.json"
    if settings_path.is_file():
        try:
            data = json.loads(settings_path.read_text(encoding="utf-8"))
            runtimes = data.get("java.configuration.runtimes", [])
            for rt in runtimes:
                if rt.get("default") and rt.get("path"):
                    p = Path(rt["path"])
                    if p.is_dir():
                        return str(p)
            for rt in runtimes:
                if rt.get("path") and Path(rt["path"]).is_dir():
                    return str(Path(rt["path"]))
        except (json.JSONDecodeError, OSError):
            pass
    if os.environ.get("JAVA_HOME"):
        return os.environ["JAVA_HOME"]
    return None


def resolve_java_executable(repo_root: Path, override: Optional[str]) -> tuple[str, str]:
    """Returns (java_exe_path, discovery_description)."""
    if override:
        exe = override
        if Path(exe).is_dir():
            candidate = Path(exe) / "bin" / ("java.exe" if os.name == "nt" else "java")
            if candidate.is_file():
                return str(candidate), f"--java-home override ({exe})"
        return exe, "--java-exe override"

    java_home = discover_java_home(repo_root)
    if java_home:
        exe = Path(java_home) / "bin" / ("java.exe" if os.name == "nt" else "java")
        if exe.is_file():
            return str(exe), f".vscode/settings.json java.configuration.runtimes / JAVA_HOME ({java_home})"

    # last resort: 'java' on PATH
    found = shutil.which("java")
    if found:
        return found, "java on PATH"

    raise RuntimeError(
        "Could not locate a Java executable. Set JAVA_HOME, pass --java-home, "
        "or ensure 'java' is on PATH."
    )


def get_java_version_text(java_exe: str) -> Optional[str]:
    try:
        proc = subprocess.run([java_exe, "-version"], capture_output=True, text=True, timeout=15)
        return (proc.stderr or proc.stdout or "").strip()
    except (OSError, subprocess.SubprocessError):
        return None


def check_java_version_at_least(version_text: Optional[str], minimum: int = 21) -> bool:
    if not version_text:
        return False
    m = re.search(r'version\s+"(\d+)', version_text)
    if not m:
        return False
    return int(m.group(1)) >= minimum


def build_classpath_argfile(repo_root: Path, runner_tmp: Path, m2_repo: Path) -> Path:
    """
    Builds (or refreshes) a JDK @argfile containing '-cp "<classpath>"' for this
    project (target/classes + every non-sources/javadoc jar under the local Maven
    repository). Forward slashes are used deliberately: JDK @argfiles apply
    backslash-escape processing to quoted content, which silently corrupts native
    Windows classpath entries like C:\\Users\\...\\jar (verified empirically -
    the same classpath works with '/' and fails with '\\' inside an @argfile).
    """
    classes_dir = repo_root / "target" / "classes"
    if not (classes_dir / "Main.class").is_file():
        raise RuntimeError(
            f"{classes_dir}\\Main.class not found. Build the project first "
            f"(this runner does not compile solver code)."
        )

    if not m2_repo.is_dir():
        raise RuntimeError(f"Local Maven repository not found at {m2_repo} (set M2_REPO to override).")

    jars = []
    for p in m2_repo.rglob("*.jar"):
        name = p.name
        if name.endswith("-sources.jar") or name.endswith("-javadoc.jar"):
            continue
        jars.append(str(p))
    jars.sort()
    jars.append(str(classes_dir))

    classpath = os.pathsep.join(jars)
    if os.name == "nt":
        classpath = classpath.replace("\\", "/")

    runner_tmp.mkdir(parents=True, exist_ok=True)
    argfile = runner_tmp / "classpath.args"
    content = f'-cp "{classpath}"'
    # No BOM: Java's @argfile parser does not strip one, and misreads the
    # leading token (verified empirically: a BOM-prefixed "-cp" is treated as
    # the main class name instead of an option).
    with open(argfile, "w", encoding="utf-8", newline="") as f:
        f.write(content)
    return argfile


# --------------------------------------------------------------------------------------
# Manifest handling
# --------------------------------------------------------------------------------------

def load_manifest(manifest_path: Path) -> list[dict]:
    with open(manifest_path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = [dict(r) for r in reader]
    if not rows:
        raise RuntimeError(f"Manifest {manifest_path} contains no data rows.")
    return rows


def instance_sort_key(row: dict):
    grid_w = as_int(col(row, "grid_width"))
    grid_h = as_int(col(row, "grid_height"))
    grid_area = (grid_w * grid_h) if (grid_w is not None and grid_h is not None) else None
    grid_size_str = col(row, "grid_size", default="")
    actual_density = as_float(col(row, "actual_agent_density", "actual_density", "agent_density"), default=0.0)
    total_agents = as_int(col(row, "total_agents"), default=0)
    num_ua = as_int(col(row, "num_ua"), default=0)
    seed = as_int(col(row, "seed"), default=0)
    return (
        grid_area if grid_area is not None else 1 << 30,
        grid_size_str,
        actual_density,
        total_agents,
        num_ua,
        seed,
    )


def rotate(seq: list, amount: int) -> list:
    if not seq:
        return list(seq)
    amount %= len(seq)
    return seq[amount:] + seq[:amount]


@dataclass
class PlannedRun:
    plan_position: int
    run_order_index: int
    instance_name: str
    scenario_path: str
    map_name: str
    solver: str
    seed: Optional[int]
    grid_size: Optional[str]
    requested_density: Optional[float]
    actual_density: Optional[float]
    total_agents: Optional[int]
    num_aa: Optional[int]
    num_ua: Optional[int]
    requested_ua_fraction: Optional[float]
    actual_ua_fraction: Optional[float]


def build_full_plan(manifest_rows: list[dict], solver_order_mode: str) -> list[PlannedRun]:
    """
    Builds the GLOBAL, stable deterministic plan across the *entire* manifest and
    *all five* canonical solvers (never affected by --solvers/--max-instances, so
    that plan_position stays stable across differently-filtered invocations that
    share the same --results root). plan_position is purely a display/ordering
    aid; it is NOT the campaign_id (see resolve_campaign_id) and is not part of
    the resume run key (see RUN_KEY_FIELDS).
    """
    sorted_rows = sorted(manifest_rows, key=instance_sort_key)

    plan: list[PlannedRun] = []
    plan_position = 0
    for instance_index, row in enumerate(sorted_rows):
        if solver_order_mode == "rotating":
            order = rotate(CANONICAL_SOLVERS, instance_index)
        else:
            order = list(CANONICAL_SOLVERS)

        for run_order_index, solver in enumerate(order):
            plan_position += 1
            plan.append(PlannedRun(
                plan_position=plan_position,
                run_order_index=run_order_index,
                instance_name=col(row, "instance_name"),
                scenario_path=col(row, "scenario_path"),
                map_name=col(row, "map_name"),
                solver=solver,
                seed=as_int(col(row, "seed")),
                grid_size=col(row, "grid_size"),
                requested_density=as_float(col(row, "requested_agent_density", "requested_density")),
                actual_density=as_float(col(row, "actual_agent_density", "actual_density", "agent_density")),
                total_agents=as_int(col(row, "total_agents")),
                num_aa=as_int(col(row, "num_aa")),
                num_ua=as_int(col(row, "num_ua")),
                requested_ua_fraction=as_float(col(row, "requested_ua_fraction")),
                actual_ua_fraction=as_float(col(row, "actual_ua_fraction")),
            ))
    return plan


# --------------------------------------------------------------------------------------
# Execution of a single (instance, solver) subprocess
# --------------------------------------------------------------------------------------

def instance_id_for(instance_name: str) -> str:
    return Path(instance_name).stem


def run_dir_name(instance_name: str, solver: str, timeout_ms: int, java_heap: str,
                 manifest_sha256: Optional[str]) -> str:
    """
    Short, filesystem-safe, deterministic raw-output directory name.

    Long generated instance names (positive_mua in particular) combined with
    a deep repo path routinely pushed res/log paths past Windows' ~260-char
    MAX_PATH, causing plain os.open()/Path.glob() calls to fail with
    FileNotFoundError even though the JVM (which uses a different, long-path-
    capable Win32 API under the hood) had written the file successfully. This
    must not depend on the Windows registry's LongPathsEnabled setting, so
    instead the directory name itself is kept short: a truncated, sanitized
    instance prefix (for human readability) plus a sha1 digest of the full
    run key (instance_name, solver, timeout_ms, java_heap, manifest_sha256),
    which is what actually guarantees uniqueness - the truncated prefix alone
    is not depended upon for collision-avoidance. The full instance_name is
    never lost: it is always written out in full in campaign_runs.csv.
    """
    digest_input = f"{instance_name}|{solver}|{timeout_ms}|{java_heap}|{manifest_sha256}"
    digest = hashlib.sha1(digest_input.encode("utf-8")).hexdigest()[:12]
    prefix = re.sub(r"[^A-Za-z0-9]+", "-", instance_id_for(instance_name)).strip("-")[:20]
    return f"{prefix}__{digest}__t{timeout_ms}_h{java_heap}"


def clean_dir(path: Path):
    if path.exists():
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def find_all(dir_path: Path, prefix: str) -> list[Path]:
    try:
        return sorted(dir_path.glob(f"{prefix} *.csv"))
    except OSError:
        # Defensive only: with run_dir_name's short paths this should not happen,
        # but a run must never crash the whole campaign over a directory listing
        # failure either. Treat it exactly like "found nothing".
        return []


def parse_result_csv(res_path: Path) -> tuple[Optional[dict], int]:
    """
    Returns (row_dict_or_None, num_data_rows). Raises FileNotFoundError if
    res_path cannot be opened, or OSError/UnicodeDecodeError/csv.Error if it
    exists but cannot be read/parsed as CSV - callers must catch these (see
    classify_solved_row) rather than let them propagate, since one run's
    result file being missing/unreadable must never abort the whole campaign.
    """
    with open(res_path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = [dict(r) for r in reader]
    if len(rows) == 1:
        return rows[0], 1
    return None, len(rows)


_EMPTY_SOLVE_FIELDS = {"solved": "", "valid": "", "timed_out": "", "mua": "", "solver_elapsed_ms": ""}


def classify_solved_row(res_path: Path) -> dict:
    """
    Reads the single result CSV for a run and returns its classification
    fields (status/error_type/error_message plus solved/valid/timed_out/mua/
    solver_elapsed_ms). Never raises: a missing, unreadable, or malformed
    result file becomes its own ERROR-status dict instead of propagating and
    crashing the whole campaign (see parse_result_csv).
    """
    try:
        row, n_rows = parse_result_csv(res_path)
    except FileNotFoundError:
        return {
            "status": "ERROR", "error_type": "MISSING_RESULT_FILE",
            "error_message": f"result CSV was listed by directory scan but is missing/inaccessible: {res_path}",
            **_EMPTY_SOLVE_FIELDS,
        }
    except (OSError, UnicodeDecodeError, csv.Error) as e:
        return {
            "status": "ERROR", "error_type": "MALFORMED_RESULT_CSV",
            "error_message": f"result CSV could not be read/parsed ({type(e).__name__}): {e}",
            **_EMPTY_SOLVE_FIELDS,
        }

    if row is None:
        return {
            "status": "ERROR", "error_type": "UNEXPECTED_ROW_COUNT",
            "error_message": f"expected exactly 1 data row in {res_path.name}, found {n_rows}",
            **_EMPTY_SOLVE_FIELDS,
        }

    solved_raw = as_int(row.get("Solved"))
    valid_raw = as_int(row.get("Valid"))
    timed_out_raw = as_int(row.get("Timed Out"))
    fields = {
        "mua": row.get("MUA", ""),
        "solver_elapsed_ms": row.get("Elapsed Time (ms)", ""),
        "solved": solved_raw if solved_raw is not None else "",
        "valid": valid_raw if valid_raw is not None else "",
        "timed_out": timed_out_raw if timed_out_raw is not None else "",
    }

    if solved_raw == 1:
        if "Valid" in row and valid_raw != 1:
            return {**fields, "status": "ERROR", "error_type": "INVALID_SOLUTION",
                    "error_message": "solver reported Solved=1 but Valid=0"}
        return {**fields, "status": "SOLVED", "error_type": "", "error_message": ""}

    if "Timed Out" not in row:
        # Reporting-only field missing entirely: surface as a diagnosable anomaly
        # rather than silently guessing UNSOLVED.
        return {**fields, "status": "ERROR", "error_type": "MISSING_TIMEOUT_FIELD",
                "error_message": "result row has no 'Timed Out' column (A_Solver reporting patch not in effect?)"}

    if timed_out_raw == 1:
        return {**fields, "status": "TIMEOUT", "error_type": "SOLVER_TIMEOUT",
                "error_message": "solver reported Timed Out=1 (Solved=0)"}

    return {**fields, "status": "UNSOLVED", "error_type": "", "error_message": ""}


def run_one(java_exe: str, classpath_argfile: Path, repo_root: Path, plan: PlannedRun,
            out_dir: Path, timeout_ms: int, grace_ms: int, java_heap: str,
            verbose: bool) -> dict:
    scen_abs = (repo_root / plan.scenario_path).resolve()
    scen_dir = scen_abs.parent

    if not scen_abs.is_file():
        return {
            "status": "ERROR", "process_return_code": None, "process_wall_time_ms": 0,
            "result_csv": "", "log_csv": "", "stdout_path": "", "stderr_path": "",
            "solved": "", "valid": "", "timed_out": "", "mua": "", "solver_elapsed_ms": "",
            "error_type": "MISSING_SCENARIO_FILE",
            "error_message": f"scenario file not found on disk: {scen_abs}",
            "started_at": now_iso(), "finished_at": now_iso(),
        }

    clean_dir(out_dir)
    res_prefix = "run"
    iregex = "^" + re.escape(plan.instance_name) + "$"
    watchdog_seconds = (timeout_ms + grace_ms) / 1000.0

    cmd = [
        java_exe,
        f"@{classpath_argfile}",
        "-ea",
        f"-Xmx{java_heap}",
        "Main",
        "-iDir", str(scen_dir),
        "-iRegex", iregex,
        "-a", str(plan.total_agents),
        "-s", plan.solver,
        "-t", str(timeout_ms),
        "-resDir", str(out_dir),
        "-resPref", res_prefix,
    ]

    started_at = now_iso()
    t0 = time.monotonic()
    return_code = None
    stdout_text = ""
    stderr_text = ""
    watchdog_killed = False
    try:
        proc = subprocess.run(
            cmd, cwd=str(repo_root),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace",
            timeout=watchdog_seconds,
        )
        return_code = proc.returncode
        stdout_text = proc.stdout or ""
        stderr_text = proc.stderr or ""
    except subprocess.TimeoutExpired as te:
        watchdog_killed = True
        stdout_text = te.stdout or ""
        stderr_text = te.stderr or ""
        if isinstance(stdout_text, bytes):
            stdout_text = stdout_text.decode("utf-8", errors="replace")
        if isinstance(stderr_text, bytes):
            stderr_text = stderr_text.decode("utf-8", errors="replace")
    finally:
        wall_ms = int((time.monotonic() - t0) * 1000)
    finished_at = now_iso()

    stdout_path = out_dir / "stdout.txt"
    stderr_path = out_dir / "stderr.txt"
    stdout_path.write_text(stdout_text, encoding="utf-8", errors="replace")
    stderr_path.write_text(stderr_text, encoding="utf-8", errors="replace")

    if verbose:
        if stdout_text.strip():
            print("----- stdout -----")
            print(stdout_text)
        if stderr_text.strip():
            print("----- stderr -----")
            print(stderr_text)

    combined_output = stdout_text + "\n" + stderr_text
    oom_hit = any(p in combined_output for p in OOM_PATTERNS)

    res_files = find_all(out_dir, "res")
    log_files = find_all(out_dir, "log")
    res_path = res_files[0] if len(res_files) == 1 else None
    log_path = log_files[0] if len(log_files) == 1 else None

    result_csv_rel = str(res_path.relative_to(repo_root)) if res_path else ""
    log_csv_rel = str(log_path.relative_to(repo_root)) if log_path else ""

    solved = valid = timed_out = mua = solver_elapsed_ms = ""
    error_type = error_message = ""
    status: str

    # Status classification priority:
    #   1. explicit JVM OOM evidence in captured output       -> OOM
    #   2. external watchdog had to kill the process           -> TIMEOUT (WATCHDOG_TIMEOUT)
    #   3. nonzero process exit with no OOM evidence            -> ERROR
    #   4. normal exit, result CSV missing/unreadable/malformed -> ERROR (never raises - see classify_solved_row)
    #   5. normal exit, result row present:
    #        Solved=1                       -> SOLVED (unless Valid=0 -> ERROR)
    #        Solved=0, Timed Out=1          -> TIMEOUT   (explicit A_Solver.abortedForTimeout flag)
    #        Solved=0, Timed Out=0          -> UNSOLVED  (never auto-labelled UNSAT)
    # No heuristic on elapsed-time-vs-timeout is used anywhere below.
    if oom_hit:
        status = "OOM"
        error_type = "OOM"
        error_message = _tail(stderr_text or stdout_text, 2000)
    elif watchdog_killed:
        status = "TIMEOUT"
        error_type = "WATCHDOG_TIMEOUT"
        error_message = f"process exceeded timeout_ms+grace_ms ({timeout_ms}+{grace_ms}ms) and was terminated"
    elif return_code != 0:
        status = "ERROR"
        error_type = "NONZERO_EXIT"
        error_message = _tail(stderr_text or stdout_text, 2000)
    elif len(res_files) == 0:
        status = "ERROR"
        error_type = "MISSING_RESULT_CSV"
        error_message = "process exited 0 but produced no res*.csv (regex likely matched 0 instances)"
    elif len(res_files) > 1:
        status = "ERROR"
        error_type = "MULTIPLE_RESULT_CSV"
        error_message = f"expected exactly one res*.csv, found {len(res_files)}: {[p.name for p in res_files]}"
    else:
        classified = classify_solved_row(res_path)
        status = classified["status"]
        error_type = classified["error_type"]
        error_message = classified["error_message"]
        solved = classified["solved"]
        valid = classified["valid"]
        timed_out = classified["timed_out"]
        mua = classified["mua"]
        solver_elapsed_ms = classified["solver_elapsed_ms"]

    return {
        "status": status,
        "process_return_code": return_code if return_code is not None else "",
        "process_wall_time_ms": wall_ms,
        "result_csv": result_csv_rel,
        "log_csv": log_csv_rel,
        "stdout_path": str(stdout_path.relative_to(repo_root)),
        "stderr_path": str(stderr_path.relative_to(repo_root)),
        "solved": solved, "valid": valid, "timed_out": timed_out,
        "mua": mua, "solver_elapsed_ms": solver_elapsed_ms,
        "error_type": error_type, "error_message": error_message,
        "started_at": started_at, "finished_at": finished_at,
    }


def _tail(text: str, n_chars: int) -> str:
    text = text.strip()
    return text[-n_chars:] if len(text) > n_chars else text


# --------------------------------------------------------------------------------------
# campaign_runs.csv / campaign_summary.csv persistence
# --------------------------------------------------------------------------------------

def compute_run_key(row: dict) -> str:
    """
    The stable identity of one experimental run: same (instance, solver) run
    under a different timeout/heap/manifest is a DIFFERENT run, not a resumable
    duplicate of an earlier one. Computed identically for freshly-planned rows
    and rows loaded back from an existing campaign_runs.csv, so lookups agree.
    """
    return "|".join(f"{f}={row.get(f)}" for f in RUN_KEY_FIELDS)


def load_existing_runs(csv_path: Path) -> dict[str, dict]:
    if not csv_path.is_file():
        return {}
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        rows = [dict(r) for r in reader]
    return {compute_run_key(r): r for r in rows}


def resolve_campaign_id(results_root: Path, existing_rows: list[dict], metadata_path: Path) -> str:
    """
    campaign_id identifies the whole campaign (this --results directory), not
    an individual row, and must stay identical across every resumed/repeated
    invocation against the same directory. Preference order: an already-
    persisted row's campaign_id, then campaign_metadata.json, then a fresh
    "<directory-name>_<timestamp>" generated once and then reused forever.
    """
    for r in existing_rows:
        cid = r.get("campaign_id")
        if cid:
            return cid
    if metadata_path.is_file():
        try:
            data = json.loads(metadata_path.read_text(encoding="utf-8"))
            cid = data.get("campaign_id")
            if cid:
                return cid
        except (json.JSONDecodeError, OSError):
            pass
    base_name = results_root.parent.name or results_root.name
    return f"{base_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"


def next_run_id_counter(existing_rows: list[dict]) -> int:
    max_id = 0
    for r in existing_rows:
        v = as_int(r.get("run_id"))
        if v is not None and v > max_id:
            max_id = v
    return max_id + 1


def should_skip(existing: Optional[dict], args) -> bool:
    if existing is None:
        return False
    if args.force:
        return False
    status = existing.get("status", "")
    if status not in TERMINAL_STATUSES:
        return False  # not a terminal/complete record -> (re)run it
    if status == "ERROR" and args.retry_errors:
        return False
    if status == "TIMEOUT" and args.retry_timeouts:
        return False
    if status == "OOM" and args.retry_oom:
        return False
    return True


def plan_row_to_run_row(plan: PlannedRun, timeout_ms: int, grace_ms: int, java_heap: str,
                         manifest_sha256: str) -> dict:
    row = {
        "plan_position": plan.plan_position,
        "run_order_index": plan.run_order_index,
        "instance_name": plan.instance_name,
        "scenario_path": plan.scenario_path,
        "map_name": plan.map_name,
        "solver": plan.solver,
        "seed": plan.seed,
        "grid_size": plan.grid_size,
        "requested_density": plan.requested_density,
        "actual_density": plan.actual_density,
        "total_agents": plan.total_agents,
        "num_aa": plan.num_aa,
        "num_ua": plan.num_ua,
        "requested_ua_fraction": plan.requested_ua_fraction,
        "actual_ua_fraction": plan.actual_ua_fraction,
        "timeout_ms": timeout_ms,
        "grace_ms": grace_ms,
        "java_heap": java_heap,
        "manifest_sha256": manifest_sha256,
    }
    row["run_key"] = compute_run_key(row)
    return row


def compute_summary(all_rows: list[dict], solvers_of_interest: list[str]) -> list[dict]:
    summary = []
    by_solver: dict[str, list[dict]] = {s: [] for s in solvers_of_interest}
    for r in all_rows:
        s = r.get("solver")
        if s in by_solver:
            by_solver[s].append(r)
    for s in solvers_of_interest:
        rows = by_solver[s]
        completed = [r for r in rows if r.get("status") in TERMINAL_STATUSES]
        n_solved = sum(1 for r in completed if r.get("status") == "SOLVED")
        n_timeout = sum(1 for r in completed if r.get("status") == "TIMEOUT")
        n_oom = sum(1 for r in completed if r.get("status") == "OOM")
        n_unsolved = sum(1 for r in completed if r.get("status") == "UNSOLVED")
        n_error = sum(1 for r in completed if r.get("status") == "ERROR")
        expected = len(rows)
        success_rate = (n_solved / expected) if expected else 0.0
        summary.append({
            "solver": s, "expected": expected, "completed": len(completed),
            "solved": n_solved, "timeout": n_timeout, "oom": n_oom,
            "unsolved": n_unsolved, "error": n_error,
            "success_rate": f"{success_rate:.4f}",
        })
    return summary


# --------------------------------------------------------------------------------------
# Metadata
# --------------------------------------------------------------------------------------

def git_metadata(repo_root: Path) -> dict:
    def run_git(args):
        try:
            p = subprocess.run(["git", *args], cwd=str(repo_root), capture_output=True,
                                text=True, timeout=15)
            if p.returncode == 0:
                return p.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            pass
        return None

    commit = run_git(["rev-parse", "HEAD"])
    status = run_git(["status", "--porcelain"])
    dirty = (status is not None and len(status) > 0) if status is not None else None
    return {"git_commit": commit, "git_dirty": dirty}


def build_metadata(args, repo_root: Path, manifest_path: Path, java_exe: str,
                    java_version_text: Optional[str], expected_runs_full: int,
                    campaign_id: str, manifest_sha256: Optional[str]) -> dict:
    gm = git_metadata(repo_root)
    self_path = Path(__file__).resolve()
    return {
        "campaign_id": campaign_id,
        "launched_at": now_iso(),
        "campaign_launch_id": str(uuid.uuid4()),
        **gm,
        "java_executable": java_exe,
        "java_version": java_version_text,
        "python_version": platform.python_version(),
        "os": platform.platform(),
        "cpu": os.environ.get("PROCESSOR_IDENTIFIER") or platform.processor() or None,
        "timeout_ms": args.timeout_ms,
        "grace_ms": args.grace_ms,
        "java_heap": args.java_heap,
        "solvers_canonical_order": CANONICAL_SOLVERS,
        "solvers_this_invocation": args.solvers,
        "solver_order_mode": args.solver_order,
        "max_instances": args.max_instances,
        "manifest_path": str(manifest_path),
        "manifest_sha256": manifest_sha256,
        "runner_script_path": str(self_path),
        "runner_script_sha256": sha256_of_file(self_path),
        "expected_runs_full_campaign": expected_runs_full,
    }


def write_metadata(metadata_path: Path, latest: dict):
    history = []
    if metadata_path.is_file():
        try:
            existing = json.loads(metadata_path.read_text(encoding="utf-8"))
            history = existing.get("launch_history", [])
        except (json.JSONDecodeError, OSError):
            history = []
    history.append(latest)
    doc = {**latest, "launch_history": history}
    atomic_write_text(metadata_path, json.dumps(doc, indent=2, default=str))


# --------------------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------------------

def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="run_mua_campaign.py",
        description="Robustly executes MAPFUA-MUA solver runs (one Java subprocess per "
                    "instance x solver pair) from a create_mua_instances.py manifest, "
                    "for later analysis with analyze_mua_results.py.",
    )
    p.add_argument("--manifest", required=True, type=Path,
                   help="Path to the generator's manifest.csv.")
    p.add_argument("--results", required=True, type=Path,
                   help="Campaign output root (raw/ and runner/ are created underneath).")
    p.add_argument("--timeout-ms", type=int, default=60000,
                   help="Solver timeout passed to Java via -t. Default: 60000.")
    p.add_argument("--grace-ms", type=int, default=5000,
                   help="Extra time allowed past --timeout-ms before the external watchdog "
                        "kills a hung process. Default: 5000.")
    p.add_argument("--java-heap", default="8g",
                   help="Fixed -Xmx heap size given to every subprocess, e.g. 8g. Default: 8g.")
    p.add_argument("--solvers", nargs="+", default=list(CANONICAL_SOLVERS),
                   metavar="SOLVER",
                   help="Subset of solvers to run this invocation (does not change each "
                        f"instance's rotation/plan_position). Choices: {', '.join(CANONICAL_SOLVERS)}. "
                        "Default: all five.")
    p.add_argument("--solver-order", choices=["fixed", "rotating"], default="rotating",
                   help="Per-instance solver execution order. Default: rotating.")
    p.add_argument("--max-instances", type=int, default=None,
                   help="Only consider the first N manifest instances (after deterministic "
                        "sorting). Default: all.")
    p.add_argument("--dry-run", action="store_true",
                   help="Print the planned campaign and exact Java command template; "
                        "launch no subprocesses.")
    p.add_argument("--verbose", action="store_true",
                   help="Echo each subprocess's captured stdout/stderr to the console.")
    p.add_argument("--force", action="store_true",
                   help="Re-run every planned (instance, solver) pair, ignoring any "
                        "existing terminal record.")
    p.add_argument("--retry-errors", action="store_true", help="Re-run rows previously recorded as ERROR.")
    p.add_argument("--retry-timeouts", action="store_true", help="Re-run rows previously recorded as TIMEOUT.")
    p.add_argument("--retry-oom", action="store_true", help="Re-run rows previously recorded as OOM.")
    p.add_argument("--java-home", default=None,
                   help="Override JDK home directory (bin/java[.exe] must exist under it). "
                        "Default: discovered from .vscode/settings.json, then JAVA_HOME, "
                        "then 'java' on PATH.")
    p.add_argument("--java-exe", default=None,
                   help="Override the exact java executable path (takes precedence over "
                        "--java-home / auto-discovery).")

    args = p.parse_args(argv)

    unknown = [s for s in args.solvers if s not in CANONICAL_SOLVERS]
    if unknown:
        p.error(f"Unknown solver(s) in --solvers: {unknown}. "
                f"Valid solvers: {', '.join(CANONICAL_SOLVERS)}")
    return args


# --------------------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------------------

def main(argv=None) -> int:
    args = parse_args(argv)
    script_dir = Path(__file__).resolve().parent
    repo_root = find_repo_root(script_dir)

    manifest_path = args.manifest if args.manifest.is_absolute() else (repo_root / args.manifest)
    manifest_path = manifest_path.resolve()
    results_root = args.results if args.results.is_absolute() else (repo_root / args.results)
    results_root = results_root.resolve()

    manifest_rows = load_manifest(manifest_path)
    full_plan = build_full_plan(manifest_rows, args.solver_order)

    # Determine eligible instances (post --max-instances), preserving the same
    # deterministic sort used to build the full plan.
    sorted_rows = sorted(manifest_rows, key=instance_sort_key)
    eligible_instance_names = [col(r, "instance_name") for r in sorted_rows]
    if args.max_instances is not None:
        eligible_instance_names = eligible_instance_names[:args.max_instances]
    eligible_instance_set = set(eligible_instance_names)

    solvers_of_interest = [s for s in CANONICAL_SOLVERS if s in args.solvers]
    invocation_plan = [
        pr for pr in full_plan
        if pr.instance_name in eligible_instance_set and pr.solver in solvers_of_interest
    ]

    expected_runs_full = len(full_plan)
    expected_runs_this_invocation = len(invocation_plan)

    java_exe, discovery_desc = resolve_java_executable(repo_root, args.java_exe or args.java_home)
    java_version_text = get_java_version_text(java_exe)

    if args.dry_run:
        print("=== DRY RUN: no Java subprocesses will be launched ===")
        print(f"Repository root      : {repo_root}")
        print(f"Manifest             : {manifest_path}")
        print(f"Scenarios (instances) : {len(sorted_rows)}")
        print(f"Solvers (canonical)   : {len(CANONICAL_SOLVERS)} -> {', '.join(CANONICAL_SOLVERS)}")
        print(f"Full campaign size    : {len(sorted_rows)} instances x {len(CANONICAL_SOLVERS)} solvers "
              f"= {expected_runs_full} expected runs")
        print()
        print(f"This invocation's filters:")
        print(f"  --max-instances     : {args.max_instances!r} -> {len(eligible_instance_set)} instance(s)")
        print(f"  --solvers           : {solvers_of_interest}")
        print(f"  --solver-order      : {args.solver_order}")
        print(f"  => planned runs this invocation = {expected_runs_this_invocation}")
        print()
        print(f"Output root           : {results_root}")
        print(f"Timeout / grace / heap: {args.timeout_ms}ms / {args.grace_ms}ms / {args.java_heap}")
        print(f"Java executable       : {java_exe}")
        print(f"Java discovery        : {discovery_desc}")
        print(f"Java version          : {(java_version_text or 'UNKNOWN').splitlines()[0] if java_version_text else 'UNKNOWN'}")
        print()
        print("Exact Java command template (one subprocess per instance x solver):")
        print(f'  "{java_exe}" @<classpath.args> -ea -Xmx{args.java_heap} Main '
              f'-iDir <scenario_parent_dir> -iRegex "^<escaped_instance_name>$" '
              f'-a <total_agents> -s <SOLVER> -t {args.timeout_ms} '
              f'-resDir <results>\\raw\\<SOLVER>\\<short-prefix>__<sha1-12>__t{args.timeout_ms}_h{args.java_heap} '
              f'-resPref run')
        print()
        print(f"First {min(10, len(invocation_plan))} planned run(s):")
        for pr in invocation_plan[:10]:
            print(f"  [{pr.plan_position:>4}] {pr.solver:<28} | {pr.instance_name}")
        return 0

    if not check_java_version_at_least(java_version_text, 21):
        eprint(f"WARNING: could not confirm Java >= 21 from discovered executable {java_exe} "
               f"(version output: {java_version_text!r}). This project's compiled classes "
               f"require Java 21+; runs may fail with UnsupportedClassVersionError.")

    runner_dir = results_root / "runner"
    raw_dir = results_root / "raw"
    runner_tmp = repo_root / "Experiments" / "MUA" / ".runner_tmp"
    runs_csv_path = runner_dir / "campaign_runs.csv"
    summary_csv_path = runner_dir / "campaign_summary.csv"
    metadata_path = runner_dir / "campaign_metadata.json"

    manifest_sha256 = sha256_of_file(manifest_path)
    classpath_argfile = build_classpath_argfile(repo_root, runner_tmp, Path.home() / ".m2" / "repository")

    existing_by_key = load_existing_runs(runs_csv_path)
    existing_rows_list = list(existing_by_key.values())
    # campaign_id identifies the whole --results directory and must stay stable
    # across every invocation that (re)uses it; run_id is the per-row identity.
    campaign_id = resolve_campaign_id(results_root, existing_rows_list, metadata_path)
    run_id_counter = next_run_id_counter(existing_rows_list)

    metadata = build_metadata(args, repo_root, manifest_path, java_exe, java_version_text,
                               expected_runs_full, campaign_id, manifest_sha256)
    runner_dir.mkdir(parents=True, exist_ok=True)
    write_metadata(metadata_path, metadata)

    # Full durable record set: everything previously known, keyed by run_key (see
    # RUN_KEY_FIELDS) -- NOT by (instance, solver) alone, so the same pair run
    # under a different timeout/heap/manifest is tracked as a distinct row.
    all_rows_by_key: dict[str, dict] = dict(existing_by_key)

    total = len(invocation_plan)
    n_executed = 0
    n_skipped = 0

    for i, pr in enumerate(invocation_plan, start=1):
        base_row = plan_row_to_run_row(pr, args.timeout_ms, args.grace_ms, args.java_heap, manifest_sha256)
        key = base_row["run_key"]
        existing = existing_by_key.get(key)

        if should_skip(existing, args):
            n_skipped += 1
            print(f"[{i}/{total}] {pr.solver:<28} | {pr.instance_name} | SKIP (already {existing.get('status')})")
            continue

        base_row["campaign_id"] = campaign_id
        if existing is not None and existing.get("run_id"):
            base_row["run_id"] = existing["run_id"]  # retry of the identical run_key: keep its row identity
        else:
            base_row["run_id"] = run_id_counter
            run_id_counter += 1

        # Short, hash-based directory name: config (timeout/heap/manifest) is part
        # of it so re-running the same instance+solver under a different budget
        # can never clobber an earlier, still-valid run_key's raw files, and it
        # stays well under Windows' ~260-char MAX_PATH regardless of how long the
        # underlying instance_name is (see run_dir_name).
        out_dir = (raw_dir / pr.solver /
                   run_dir_name(pr.instance_name, pr.solver, args.timeout_ms, args.java_heap, manifest_sha256))

        try:
            result = run_one(
                java_exe=java_exe, classpath_argfile=classpath_argfile, repo_root=repo_root,
                plan=pr, out_dir=out_dir, timeout_ms=args.timeout_ms, grace_ms=args.grace_ms,
                java_heap=args.java_heap, verbose=args.verbose,
            )
        except Exception as e:  # noqa: BLE001 - last-resort safety net, see module docstring
            # run_one() already handles every failure mode it anticipates (missing
            # scenario, watchdog kill, OOM, missing/malformed result CSV) without
            # raising. This is a final backstop so that a truly unforeseen failure
            # (e.g. a permissions error creating out_dir) still degrades to a single
            # ERROR row instead of aborting the rest of the campaign.
            now = now_iso()
            result = {
                "status": "ERROR", "process_return_code": "", "process_wall_time_ms": "",
                "result_csv": "", "log_csv": "", "stdout_path": "", "stderr_path": "",
                "solved": "", "valid": "", "timed_out": "", "mua": "", "solver_elapsed_ms": "",
                "error_type": "UNEXPECTED_EXCEPTION",
                "error_message": f"{type(e).__name__}: {e}",
                "started_at": now, "finished_at": now,
            }
            eprint(f"[{i}/{total}] {pr.solver:<28} | {pr.instance_name} | UNEXPECTED_EXCEPTION: {e!r}")
        row = {**base_row, **result}
        all_rows_by_key[key] = row
        n_executed += 1

        elapsed_display = f"{row['process_wall_time_ms']} ms" if row['process_wall_time_ms'] < 10000 \
            else f"{row['process_wall_time_ms'] / 1000.0:.1f} s"
        print(f"[{i}/{total}] {pr.solver:<28} | {pr.instance_name} | {row['status']} | {elapsed_display}")

        # Persist after EVERY run (atomic replace), so a crash loses at most one run.
        ordered_rows = sorted(all_rows_by_key.values(), key=lambda r: as_int(r.get("run_id"), default=0))
        atomic_write_csv(runs_csv_path, RUN_COLUMNS, ordered_rows)

        summary_rows = compute_summary(list(all_rows_by_key.values()), CANONICAL_SOLVERS)
        atomic_write_csv(summary_csv_path, SUMMARY_COLUMNS, summary_rows)

    # Final summary write (covers the all-skipped case, where the loop above never wrote).
    ordered_rows = sorted(all_rows_by_key.values(), key=lambda r: as_int(r.get("run_id"), default=0))
    if ordered_rows:
        atomic_write_csv(runs_csv_path, RUN_COLUMNS, ordered_rows)
        summary_rows = compute_summary(list(all_rows_by_key.values()), CANONICAL_SOLVERS)
        atomic_write_csv(summary_csv_path, SUMMARY_COLUMNS, summary_rows)

    print()
    print(f"Done. {n_executed} run(s) executed, {n_skipped} run(s) skipped (already complete).")
    print(f"campaign_runs.csv   : {runs_csv_path}")
    print(f"campaign_summary.csv: {summary_csv_path}")
    print(f"campaign_metadata.json: {metadata_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
