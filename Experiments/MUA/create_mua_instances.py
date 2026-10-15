#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate MAPFUA-MUA experiment instances.

Creates .map/.scen pairs in the same family directory plus a manifest.csv containing experiment metadata.\n\nScenario filenames intentionally begin with the exact map stem followed by '-random'. The project's InstanceBuilder_MovingAI uses that convention to pair each .scen with its .map.
UAs are encoded exactly as in the existing project: goal == start.

Experiment modes:
  1) ua_scale       fixed AA, vary UA independently
  2) aa_scale       fixed UA, vary AA independently
  3) density        fixed AA:UA ratio, vary total agent density
  4) ratio          fixed total population, vary AA/UA split (legacy/control)
  5) corridor       one mandatory blocker; vary corridor length; MUA*=1
  6) k_blockers     k independent mandatory blockers; MUA*=k
  7) decoy          one mandatory blocker + irrelevant UAs; MUA*=1
  8) decoy_fixed    fixed map/topology, one mandatory blocker + varying irrelevant UAs; MUA*=1
  9) unsolvable     corridor blocker with no escape pocket; known unsolvable
 10) all_constructed
 11) matrix         factorial benchmark: grid size x agent density x UA/AA ratio x seed.
                     Generates its own open MovingAI grids. Total agent count is fixed
                     within a grid+density group; only the UA/AA split changes across
                     ratio points, using a nested/paired sampling scheme (see
                     assign_matrix_agents / build_matrix_agents below).
 12) positive_mua   controlled, guaranteed-solvable instances with known, strictly
                     positive optimal MUA, for studying whether A*+OD heuristics help
                     when UA movement genuinely matters. Exactly one "primary" AA ever
                     enters the mandatory-blocker lane; any additional AAs (num_aa > 1)
                     are trivial "auxiliary" agents confined to their own separate,
                     obstacle-free row (see build_queued_lane_instance) so scaling
                     num_aa grows the joint AA state space without also making the
                     blocker lane itself combinatorially harder to search. The whole
                     map is still a single connected component (one connector cell)
                     so heuristics that assume full reachability never crash. Three
                     families, each swept over num_aa in {2,3} and 5 seeded,
                     semantics-preserving spacing variations that never change the
                     true MUA:
                       required_mua  expected_mua in {1,2,3}: every UA is a mandatory
                                     lane blocker.
                       decoy_ua      expected_mua fixed at 1, plus extra_decoy_ua in
                                     {0,2,4} irrelevant UAs sitting in unused pockets.
                       mixed         expected_mua in {2,3} combined with extra_decoy_ua
                                     in {2,4}.
 13) random_obstacle_screen
                     reproducible, UNFILTERED random-obstacle screening dataset: random
                     obstacle layout (rejection-sampled only for map connectivity, never
                     steered away from bottlenecks) x random AA/UA placement, swept over
                     obstacle density and UA count. No pre-filtering by MUA or
                     solvability, and no instance is solved during generation.
 14) random_plan    ordinary random instances for every row of an experiment plan CSV
                     (grid_size, num_aa, total_agents, num_ua, expected_random_instances),
                     seeds 1..N per configuration. Distinct starts; UAs goal == start; AA
                     goals distinct among AAs and != own start, but MAY equal another
                     agent's start. One open WxH map per grid size (or --map), stored next
                     to the .scen files; manifest paths are repo-relative with '/'.

Examples:
  python create_mua_instances.py ua_scale --map grid-15-4.map --aa 4 --ua-values 2 4 6 8 10
  python create_mua_instances.py aa_scale --map grid-15-4.map --ua 8 --aa-values 2 4 6 8
  python create_mua_instances.py corridor --length-values 7 9 11 15 21
  python create_mua_instances.py k_blockers --k-values 1 2 3 4 5
  python create_mua_instances.py decoy --ua-values 1 2 4 8 12
  python create_mua_instances.py decoy_fixed --ua-values 1 2 4 8
  python create_mua_instances.py all_constructed
  python create_mua_instances.py matrix --output Experiments/MUA/matrix/inputs ^
      --grid-sizes 5 6 8 10 12 15 20 ^
      --density-values 0.30 0.35 0.40 0.45 0.50 0.55 0.60 0.65 0.70 0.75 0.80 ^
      --ua-fractions 0.10 0.20 0.30 0.40 0.50 --seeds 30
  python create_mua_instances.py positive_mua ^
      --output Experiments/MUA/campaign/positive_mua_calibration/inputs
  py Experiments/MUA/create_mua_instances.py random_plan ^
      --plan Experiments/MUA/table_5x5/experiment_plan_5x5.csv ^
      --output Experiments/MUA/table_5x5/inputs
  python create_mua_instances.py random_obstacle_screen ^
      --output Experiments/MUA/campaign/random_obstacle_screen/inputs ^
      --grid-sizes 8 --aa 2 --ua-values 3 5 ^
      --obstacle-density-values 0.10 0.20 0.30 --seeds 10
"""

import argparse
import csv
import math
import random
import shutil
from pathlib import Path
from typing import List, Tuple

Cell = Tuple[int, int]


def load_map(path: Path):
    lines = [line.rstrip("\n") for line in path.read_text().splitlines()]
    height = int(lines[1].split()[1])
    width = int(lines[2].split()[1])
    grid = lines[4:4 + height]
    free_chars = {".", "G", "S"}
    free = [(x, y) for y in range(height) for x in range(width) if grid[y][x] in free_chars]
    return grid, free, width, height


def euclidean(a: Cell, b: Cell):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def write_map(path: Path, grid: List[str]):
    h, w = len(grid), len(grid[0])
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="\n") as f:
        f.write("type octile\n")
        f.write(f"height {h}\n")
        f.write(f"width {w}\n")
        f.write("map\n")
        for row in grid:
            f.write(row + "\n")


def write_scen(path: Path, map_name: str, width: int, height: int, agents):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="\n") as f:
        f.write("version 1\n")
        for i, agent in enumerate(agents):
            sx, sy = agent["start"]
            gx, gy = agent["goal"]
            dist = 0.0 if agent["ua"] else euclidean(agent["start"], agent["goal"])
            f.write(
                f"{i}\t{map_name}\t{width}\t{height}\t"
                f"{sx}\t{sy}\t{gx}\t{gy}\t{dist:.8f}\n"
            )


def make_random_agents(free, num_aa, num_ua, seed):
    """
    Uses distinct start cells for all agents and distinct AA goal cells not used
    as starts. This avoids accidental duplicates while still producing ordinary
    random MAPF scenarios.
    """
    rng = random.Random(seed)
    total = num_aa + num_ua
    need = total + num_aa
    if need > len(free):
        raise ValueError(
            f"Need {need} distinct cells for {num_aa} AA + {num_ua} UA, "
            f"but map has only {len(free)} free cells."
        )
    chosen = rng.sample(free, need)
    starts = chosen[:total]
    aa_goals = chosen[total:]
    agents = []
    for i in range(num_aa):
        agents.append({"start": starts[i], "goal": aa_goals[i], "ua": False})
    for i in range(num_ua):
        s = starts[num_aa + i]
        agents.append({"start": s, "goal": s, "ua": True})
    return agents


def append_manifest(records, scen_path, map_path, family, parameter, seed,
                    num_aa, num_ua, free_cells, expected_mua, known_solvable):
    records.append({
        "instance_name": scen_path.name,
        "scenario_path": str(scen_path),
        "map_name": map_path.name,
        "map_path": str(map_path),
        "instance_family": family,
        "family_parameter": parameter,
        "seed": seed,
        "num_aa": num_aa,
        "num_ua": num_ua,
        "total_agents": num_aa + num_ua,
        "free_cells": free_cells,
        "agent_density": (num_aa + num_ua) / free_cells if free_cells else "",
        "expected_mua": "" if expected_mua is None else expected_mua,
        "known_solvable": known_solvable,
    })


def random_sweep(args, mode, records):
    source_map_path = Path(args.map)
    grid, free, width, height = load_map(source_map_path)
    out = Path(args.output)

    def write_config(num_aa, num_ua, seed, agents, parameter):
        config_dir = out / mode / f"aa-{num_aa}-ua-{num_ua}"
        config_dir.mkdir(parents=True, exist_ok=True)

        # Java MovingAI loader expects the map beside the scenarios.
        map_path = config_dir / source_map_path.name
        if source_map_path.resolve() != map_path.resolve():
            shutil.copy2(source_map_path, map_path)

        name = (
            f"{map_path.stem}-random-{mode}-"
            f"aa-{num_aa}-ua-{num_ua}-seed-{seed}.scen"
        )
        scen = config_dir / name

        write_scen(
            scen,
            map_path.name,
            width,
            height,
            agents
        )

        append_manifest(
            records,
            scen,
            map_path,
            mode,
            parameter,
            seed,
            num_aa,
            num_ua,
            len(free),
            None,
            "unknown"
        )

    if mode == "ua_scale":
        # Controlled / paired sweep:
        # for a given seed, all configurations use exactly the same AAs.
        # Larger UA configurations extend the same UA prefix.
        num_aa = args.aa
        ua_values = sorted(set(args.ua_values))
        max_ua = max(ua_values)

        need = num_aa + max_ua + num_aa
        if need > len(free):
            raise ValueError(
                f"Need {need} distinct cells, but map has only "
                f"{len(free)} free cells."
            )

        for seed in range(1, args.seeds + 1):
            rng = random.Random(seed)
            chosen = rng.sample(free, need)

            aa_starts = chosen[:num_aa]
            ua_starts = chosen[num_aa:num_aa + max_ua]
            aa_goals = chosen[num_aa + max_ua:]

            for num_ua in ua_values:
                agents = []

                for i in range(num_aa):
                    agents.append({
                        "start": aa_starts[i],
                        "goal": aa_goals[i],
                        "ua": False
                    })

                for i in range(num_ua):
                    s = ua_starts[i]
                    agents.append({
                        "start": s,
                        "goal": s,
                        "ua": True
                    })

                write_config(
                    num_aa,
                    num_ua,
                    seed,
                    agents,
                    num_ua
                )

    elif mode == "aa_scale":
        # Same idea for AA scaling:
        # fixed UA set and nested AA sets for each seed.
        aa_values = sorted(set(args.aa_values))
        max_aa = max(aa_values)
        num_ua = args.ua

        need = max_aa + num_ua + max_aa
        if need > len(free):
            raise ValueError(
                f"Need {need} distinct cells, but map has only "
                f"{len(free)} free cells."
            )

        for seed in range(1, args.seeds + 1):
            rng = random.Random(seed)
            chosen = rng.sample(free, need)

            aa_starts = chosen[:max_aa]
            ua_starts = chosen[max_aa:max_aa + num_ua]
            aa_goals = chosen[max_aa + num_ua:]

            for num_aa in aa_values:
                agents = []

                for i in range(num_aa):
                    agents.append({
                        "start": aa_starts[i],
                        "goal": aa_goals[i],
                        "ua": False
                    })

                for s in ua_starts:
                    agents.append({
                        "start": s,
                        "goal": s,
                        "ua": True
                    })

                write_config(
                    num_aa,
                    num_ua,
                    seed,
                    agents,
                    num_aa
                )

    elif mode == "ratio":
        configs = [
            (aa, args.total_agents - aa)
            for aa in args.aa_values
        ]

        for seed in range(1, args.seeds + 1):
            for num_aa, num_ua in configs:
                agents = make_random_agents(
                    free, num_aa, num_ua, seed
                )
                write_config(
                    num_aa, num_ua, seed,
                    agents, num_aa
                )

    elif mode == "density":
        configs = []

        for density in args.density_values:
            total = max(2, int(round(density * len(free))))
            num_aa = max(
                1,
                int(round(total * args.aa_fraction))
            )
            num_ua = max(0, total - num_aa)
            configs.append((num_aa, num_ua))

        for seed in range(1, args.seeds + 1):
            for num_aa, num_ua in configs:
                agents = make_random_agents(
                    free, num_aa, num_ua, seed
                )

                write_config(
                    num_aa,
                    num_ua,
                    seed,
                    agents,
                    (num_aa + num_ua) / len(free)
                )

    else:
        raise ValueError(mode)
    
def corridor_grid(length, pockets):
    if length < 5:
        raise ValueError("Corridor length must be at least 5.")
    # Row 1 is the corridor. Row 0 contains optional one-cell escape pockets.
    top = ["@"] * length
    for x in pockets:
        top[x] = "."
    return ["".join(top), "." * length, "@" * length]


def constructed_corridor(args, records):
    out = Path(args.output)
    for length in args.length_values:
        center = length // 2
        grid = corridor_grid(length, [center])
        map_path = out / "corridor" / f"corridor-L-{length}.map"
        write_map(map_path, grid)

        agents = [
            {"start": (0, 1), "goal": (length - 1, 1), "ua": False},
            {"start": (center, 1), "goal": (center, 1), "ua": True},
        ]
        scen = out / "corridor" / f"{map_path.stem}-random-aa-1-ua-1-seed-1.scen"
        write_scen(scen, map_path.name, length, 3, agents)
        free = length + 1
        append_manifest(records, scen, map_path, "corridor", length, 1, 1, 1, free, 1, True)


def constructed_k_blockers(args, records):
    out = Path(args.output)
    for k in args.k_values:
        length = max(7, 4 * k + 3)
        xs = [2 + 4 * i for i in range(k)]
        grid = corridor_grid(length, xs)
        map_path = out / "k_blockers" / f"k_blockers-k-{k}.map"
        write_map(map_path, grid)

        agents = [{"start": (0, 1), "goal": (length - 1, 1), "ua": False}]
        for x in xs:
            agents.append({"start": (x, 1), "goal": (x, 1), "ua": True})

        scen = out / "k_blockers" / f"{map_path.stem}-random-aa-1-ua-{k}-seed-1.scen"
        write_scen(scen, map_path.name, length, 3, agents)
        free = length + k
        append_manifest(records, scen, map_path, "k_blockers", k, 1, 1, k, free, k, True)


def constructed_decoy(args, records):
    """
    Exactly one UA is on the corridor and must move. Remaining UAs start in
    side pockets and are irrelevant to feasibility. Therefore expected MUA*=1.
    """
    out = Path(args.output)
    for total_ua in args.ua_values:
        if total_ua < 1:
            continue
        decoys = total_ua - 1
        length = max(9, 2 * total_ua + 5)
        center = length // 2

        # Enough pockets for the mandatory blocker + every decoy.
        candidate_x = [x for x in range(1, length - 1) if x != center]
        pocket_x = [center] + candidate_x[:decoys]
        grid = corridor_grid(length, pocket_x)
        map_path = out / "decoy" / f"decoy-ua-{total_ua}.map"
        write_map(map_path, grid)

        agents = [
            {"start": (0, 1), "goal": (length - 1, 1), "ua": False},
            {"start": (center, 1), "goal": (center, 1), "ua": True},
        ]
        for x in candidate_x[:decoys]:
            agents.append({"start": (x, 0), "goal": (x, 0), "ua": True})

        scen = out / "decoy" / f"{map_path.stem}-random-aa-1-ua-{total_ua}-seed-1.scen"
        write_scen(scen, map_path.name, length, 3, agents)
        free = length + len(pocket_x)
        append_manifest(
            records, scen, map_path, "decoy", total_ua, 1,
            1, total_ua, free, 1, True
        )


def constructed_decoy_fixed(args, records):
    """
    Controlled decoy family.

    All scenarios use exactly the same map/topology and the same AA start/goal.
    Exactly one UA is a mandatory corridor blocker. Additional UAs occupy
    pre-existing side pockets and are irrelevant to feasibility, so MUA*=1.

    This family isolates the effect of |UA| from map size / free-cell count.
    """
    out = Path(args.output)
    ua_values = sorted({u for u in args.ua_values if u >= 1})
    if not ua_values:
        return

    max_ua = max(ua_values)
    max_decoys = max_ua - 1

    # Choose the map once from the largest requested UA count and reuse the
    # exact same topology for every scenario in the family.
    length = max(9, 2 * max_ua + 5)
    center = length // 2
    candidate_x = [x for x in range(1, length - 1) if x != center]
    decoy_positions = candidate_x[:max_decoys]
    pocket_x = [center] + decoy_positions
    grid = corridor_grid(length, pocket_x)
    free = length + len(pocket_x)

    for total_ua in ua_values:
        map_path = out / "decoy_fixed" / f"decoy-fixed-ua-{total_ua}.map"
        write_map(map_path, grid)

        agents = [
            {"start": (0, 1), "goal": (length - 1, 1), "ua": False},
            {"start": (center, 1), "goal": (center, 1), "ua": True},
        ]
        for x in decoy_positions[: total_ua - 1]:
            agents.append({"start": (x, 0), "goal": (x, 0), "ua": True})

        scen = (
            out / "decoy_fixed" /
            f"{map_path.stem}-random-aa-1-ua-{total_ua}-seed-1.scen"
        )
        write_scen(scen, map_path.name, length, 3, agents)
        append_manifest(
            records, scen, map_path, "decoy_fixed", total_ua, 1,
            1, total_ua, free, 1, True
        )


def constructed_unsolvable(args, records):
    out = Path(args.output)
    for length in args.length_values:
        if length < 5:
            continue
        center = length // 2
        grid = ["@" * length, "." * length, "@" * length]
        map_path = out / "unsolvable" / f"unsolvable-L-{length}.map"
        write_map(map_path, grid)

        agents = [
            {"start": (0, 1), "goal": (length - 1, 1), "ua": False},
            {"start": (center, 1), "goal": (center, 1), "ua": True},
        ]
        scen = out / "unsolvable" / f"{map_path.stem}-random-aa-1-ua-1-seed-1.scen"
        write_scen(scen, map_path.name, length, 3, agents)
        append_manifest(
            records, scen, map_path, "unsolvable", length, 1,
            1, 1, length, None, False
        )


def build_queued_lane_instance(num_aa, k_blockers, extra_decoy_ua, seed):
    """
    Shared geometry for all three positive_mua families.

    Only ONE agent - the "primary" AA - ever enters the mandatory-blocker
    lane. This is deliberate: an earlier design queued every AA single-file
    through that same lane, and while it was structurally correct (true
    MUA* was still exactly k_blockers), validating it empirically showed the
    JOINT low-level search over multiple AAs sharing one 1-wide lane is
    combinatorially brutal in practice - CBSStarMUA needed ~94s and every
    A*+OD heuristic (including IH, then named H4RandomPick) still hadn't finished after
    180s on a single aa=2/mua=3 instance. That defeats the point of a
    *calibration* family. So every additional AA beyond the first is instead
    a trivial "auxiliary" agent confined to its own separate, obstacle-free
    row, which only scales the size of the joint AA state space (the actual
    research variable) without also multiplying the search depth needed to
    resolve the blocker lane itself:

        row 0: escape pockets, "." only directly above each blocker/decoy
        row 1: the primary AA's lane -
               [start x=0] -- gap -- [k_blockers, each with its own pocket
               directly above] -- gap -- [primary AA's goal, at the far end]
        row 2: solid wall, except ONE open cell at x=0 connecting row 1 to
               row 3 (present purely so every location is mutually reachable
               - DistanceTableSingleAgentHeuristic throws if a target is
               unreachable on a map declared strongly connected, which every
               MovingAI grid map is in this project - never because any
               agent's optimal path actually uses it)
        row 3: a fully open runway holding the (num_aa - 1) auxiliary AAs,
               present only when num_aa > 1

    Because row 1 is a single 1-wide lane with no other route to the
    primary AA's goal, EVERY one of the k_blockers UAs must vacate its cell
    (into its own pocket) at least once for the primary AA to pass - and
    none of them need to move more than once, or return afterward, since
    nothing else ever needs that pocket or that corridor cell again. So
    this instance's true MUA* is always exactly k_blockers: independent of
    num_aa (auxiliary AAs never touch row 0 or row 1, so they can never be
    required to move nor able to reduce how many blockers are required) and
    independent of extra_decoy_ua (each decoy sits in its own separate,
    otherwise-unused pocket cell in row 0, so it never needs to move and
    never interacts with anything either).

    `seed` perturbs gap sizes, blocker spacing, and auxiliary AA placement (a
    semantics-preserving variation: different corridor length/shape and
    different auxiliary start/goal cells, identical topology and identical
    required-move structure), so five seeds never produce byte-identical
    maps while expected_mua stays exactly k_blockers for all of them.

    Returns a dict with: grid (list[str]), width, height, free_cells, agents,
    num_aa, num_ua, total_agents.
    """
    if num_aa < 1:
        raise ValueError("num_aa must be >= 1")
    if k_blockers < 1:
        raise ValueError("k_blockers must be >= 1 (positive_mua instances require expected_mua > 0)")
    if extra_decoy_ua < 0:
        raise ValueError("extra_decoy_ua must be >= 0")

    rng = random.Random(seed)

    cursor = 1 + rng.randint(1, 3)
    blocker_xs = []
    for i in range(k_blockers):
        blocker_xs.append(cursor)
        cursor += 1 + rng.randint(2, 4)
    cursor += rng.randint(1, 3)
    primary_goal_x = cursor
    length = primary_goal_x + 1

    decoy_candidates = [x for x in range(length) if x not in blocker_xs]
    if extra_decoy_ua > len(decoy_candidates):
        raise ValueError(
            f"Corridor of length {length} has no room for {extra_decoy_ua} decoy pockets "
            f"distinct from the {k_blockers} mandatory blocker(s)."
        )
    decoy_xs = sorted(rng.sample(decoy_candidates, extra_decoy_ua))

    row0 = ["@"] * length
    for x in blocker_xs + decoy_xs:
        row0[x] = "."
    row1 = ["."] * length

    agents = [{"start": (0, 1), "goal": (primary_goal_x, 1), "ua": False}]
    for x in blocker_xs:
        agents.append({"start": (x, 1), "goal": (x, 1), "ua": True})
    for x in decoy_xs:
        agents.append({"start": (x, 0), "goal": (x, 0), "ua": True})

    num_auxiliary = num_aa - 1
    if num_auxiliary == 0:
        row2 = ["@"] * length
        grid = ["".join(row0), "".join(row1), "".join(row2)]
    else:
        row2 = ["@"] * length
        row2[0] = "."
        row3 = ["."] * length
        grid = ["".join(row0), "".join(row1), "".join(row2), "".join(row3)]

        # Distinct, seed-varied start/goal cells for the auxiliary AAs, all confined
        # to row 3 (never row 1's connector cell at x=0), preserving relative order
        # left-to-right so no two auxiliary AAs ever need to swap places.
        aux_cells = [x for x in range(1, length)]
        rng.shuffle(aux_cells)
        needed = 2 * num_auxiliary
        if needed > len(aux_cells):
            raise ValueError(
                f"Auxiliary runway of length {length - 1} has no room for {num_auxiliary} "
                f"auxiliary AA(s) needing {needed} distinct cells."
            )
        chosen = sorted(aux_cells[:needed])
        aux_starts = chosen[:num_auxiliary]
        aux_goals = chosen[num_auxiliary:]
        for start_x, goal_x in zip(aux_starts, aux_goals):
            agents.append({"start": (start_x, 3), "goal": (goal_x, 3), "ua": False})

    free_cells = sum(row.count(".") for row in grid)
    num_ua = k_blockers + extra_decoy_ua
    return {
        "grid": grid,
        "width": length,
        "height": len(grid),
        "free_cells": free_cells,
        "agents": agents,
        "num_aa": num_aa,
        "num_ua": num_ua,
        "total_agents": num_aa + num_ua,
    }


def append_positive_mua_manifest(records, *, scen_path, map_path, positive_family, family_parameter,
                                 seed, built, expected_mua, required_mua, extra_decoy_ua):
    records.append({
        "instance_name": scen_path.name,
        "scenario_path": str(scen_path),
        "map_name": map_path.name,
        "map_path": str(map_path),
        "instance_family": "positive_mua",
        "family_parameter": family_parameter,
        "seed": seed,
        "num_aa": built["num_aa"],
        "num_ua": built["num_ua"],
        "total_agents": built["total_agents"],
        "free_cells": built["free_cells"],
        "agent_density": built["total_agents"] / built["free_cells"] if built["free_cells"] else "",
        "expected_mua": expected_mua,
        "known_solvable": True,
        "required_mua": required_mua,
        "extra_decoy_ua": extra_decoy_ua,
        "positive_family": positive_family,
    })


POSITIVE_MUA_MANIFEST_FIELDS = [
    "instance_name", "scenario_path", "map_name", "map_path",
    "instance_family", "family_parameter", "seed",
    "num_aa", "num_ua", "total_agents", "free_cells", "agent_density",
    "expected_mua", "known_solvable",
    "required_mua", "extra_decoy_ua", "positive_family",
]


def _write_positive_mua_instance(out, subdir, name_prefix, built):
    config_dir = out / subdir
    map_path = config_dir / f"{name_prefix}.map"
    write_map(map_path, built["grid"])
    scen_path = config_dir / f"{map_path.stem}-random-{name_prefix}.scen"
    write_scen(scen_path, map_path.name, built["width"], built["height"], built["agents"])
    return map_path, scen_path


def positive_mua_required_family(args, records):
    """expected_mua in {1,2,3}: every required UA genuinely needs to move at least once."""
    for num_aa in args.positive_num_aa_values:
        for k in args.required_mua_values:
            for seed in range(1, args.positive_mua_seeds + 1):
                built = build_queued_lane_instance(num_aa, k, 0, seed)
                name_prefix = f"positive-required-aa{num_aa}-mua{k}-seed-{seed:03d}"
                map_path, scen_path = _write_positive_mua_instance(
                    Path(args.output), f"required_mua/aa{num_aa}-mua{k}", name_prefix, built)
                append_positive_mua_manifest(
                    records, scen_path=scen_path, map_path=map_path,
                    positive_family="required_mua", family_parameter=k, seed=seed,
                    built=built, expected_mua=k, required_mua=k, extra_decoy_ua=0,
                )


def positive_mua_decoy_family(args, records):
    """expected_mua fixed at 1: only one UA is required; extra_decoy_ua irrelevant UAs must not become required."""
    for num_aa in args.positive_num_aa_values:
        for extra_decoy_ua in args.positive_decoy_values:
            for seed in range(1, args.positive_mua_seeds + 1):
                built = build_queued_lane_instance(num_aa, 1, extra_decoy_ua, seed)
                name_prefix = f"positive-decoy-aa{num_aa}-extra{extra_decoy_ua}-seed-{seed:03d}"
                map_path, scen_path = _write_positive_mua_instance(
                    Path(args.output), f"decoy_ua/aa{num_aa}-extra{extra_decoy_ua}", name_prefix, built)
                append_positive_mua_manifest(
                    records, scen_path=scen_path, map_path=map_path,
                    positive_family="decoy_ua", family_parameter=extra_decoy_ua, seed=seed,
                    built=built, expected_mua=1, required_mua=1, extra_decoy_ua=extra_decoy_ua,
                )


def positive_mua_mixed_family(args, records):
    """expected_mua in {2,3} combined with extra_decoy_ua in {2,4}: several UAs must move, others are irrelevant."""
    for num_aa in args.positive_num_aa_values:
        for k in args.mixed_mua_values:
            for extra_decoy_ua in args.mixed_decoy_values:
                for seed in range(1, args.positive_mua_seeds + 1):
                    built = build_queued_lane_instance(num_aa, k, extra_decoy_ua, seed)
                    name_prefix = f"positive-mixed-aa{num_aa}-mua{k}-extra{extra_decoy_ua}-seed-{seed:03d}"
                    map_path, scen_path = _write_positive_mua_instance(
                        Path(args.output), f"mixed/aa{num_aa}-mua{k}-extra{extra_decoy_ua}", name_prefix, built)
                    append_positive_mua_manifest(
                        records, scen_path=scen_path, map_path=map_path,
                        positive_family="mixed", family_parameter=f"mua{k}_decoy{extra_decoy_ua}", seed=seed,
                        built=built, expected_mua=k, required_mua=k, extra_decoy_ua=extra_decoy_ua,
                    )


def positive_mua_sweep(args, records):
    positive_mua_required_family(args, records)
    positive_mua_decoy_family(args, records)
    positive_mua_mixed_family(args, records)


def save_positive_mua_manifest(output, records):
    path = Path(output) / "manifest.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=POSITIVE_MUA_MANIFEST_FIELDS)
        w.writeheader()
        w.writerows(records)
    print(f"Generated {len(records)} positive-MUA scenarios.")
    print(f"Manifest: {path}")


class GoalAssignmentInfeasible(Exception):
    """Raised when no valid AA goal cell exists for some agent in a matrix config."""


def parse_grid_size_token(token) -> Tuple[int, int]:
    """
    Accepts either a plain integer 'N' (interpreted as an NxN square grid) or a
    'WxH' token (e.g. '10x15'), so the same machinery stays usable for
    non-square grids in the future.
    """
    s = str(token).strip().lower()
    if "x" in s:
        w_str, h_str = s.split("x", 1)
        return int(w_str), int(h_str)
    n = int(s)
    return n, n


def generate_open_grid(width: int, height: int) -> List[str]:
    """Fully open (obstacle-free) MovingAI grid: every cell is traversable."""
    row = "." * width
    return [row for _ in range(height)]


def round_half_up(x: float) -> int:
    """
    Deterministic rounding policy used throughout matrix mode: round half away
    from zero (e.g. 7.5 -> 8). Applied to both total-agent counts and UA counts
    so small-grid rounding (Part 4 of the spec) is reproducible and documented
    in exactly one place.
    """
    return int(math.floor(x + 0.5))


def free_cells_connected(free_cells) -> bool:
    """
    True iff the given free cells form a single 4-connected component - matching
    the Java loader's actual connectivity model (MapFactory.newSimple4Connected
    2D_GraphMap: orthogonal neighbors only, no diagonals). Used both to accept/
    reject a randomly generated obstacle layout and to re-verify it afterward.
    """
    free_set = set(free_cells)
    if not free_set:
        return False
    start = next(iter(free_set))
    visited = {start}
    stack = [start]
    while stack:
        x, y = stack.pop()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy)
            if n in free_set and n not in visited:
                visited.add(n)
                stack.append(n)
    return len(visited) == len(free_set)


def generate_random_obstacle_grid(width: int, height: int, obstacle_density: float, rng: random.Random,
                                  max_attempts: int = 2000):
    """
    Natural/unfiltered random obstacle placement at the requested density,
    reproducible from the given rng's seed: obstacle cells are drawn uniformly
    at random (not steered away from bottlenecks - the point of this screen is
    to see what A*+OD/CBS* do on ordinary, unfiltered random maps). The only
    acceptance criterion is the hard structural requirement that the resulting
    free-cell graph be a single connected component; a disconnected draw is
    simply rejected and a fresh one drawn from the same (continuing) rng
    stream, so the whole process is still exactly reproducible from one seed.
    Returns (grid_rows, free_cells).
    """
    all_cells = [(x, y) for y in range(height) for x in range(width)]
    total = len(all_cells)
    num_obstacles = max(0, min(total - 1, round_half_up(obstacle_density * total)))
    for _ in range(max_attempts):
        obstacles = set(rng.sample(all_cells, num_obstacles))
        free = [c for c in all_cells if c not in obstacles]
        if free_cells_connected(free):
            grid = ["".join("@" if (x, y) in obstacles else "." for x in range(width)) for y in range(height)]
            return grid, free
    raise RuntimeError(
        f"Could not draw a connected {width}x{height} grid at obstacle_density={obstacle_density} "
        f"after {max_attempts} attempts from this seed's RNG stream."
    )


def realize_total_agents(free_cells: int, requested_density: float) -> Tuple[int, float]:
    """
    total_agents ~= requested_density * free_cells, rounded half-up, clamped to
    [2, free_cells] so a UA/AA split (1 <= num_ua < total_agents) is always
    representable. Returns (total_agents, actual_agent_density).
    """
    if free_cells <= 0:
        return 0, 0.0
    total = round_half_up(requested_density * free_cells)
    total = max(2, min(free_cells, total))
    return total, total / free_cells


def realize_ua_fractions(total_agents: int, requested_fractions):
    """
    Maps each requested UA fraction to a concrete, deterministic num_ua via
    round_half_up, clamped to 1 <= num_ua < total_agents. When two or more
    requested fractions collapse onto the same num_ua (common on small grids),
    only the first-seen one is realized; the rest are reported as collisions
    instead of silently creating duplicate experimental points.

    Returns (realized, collisions):
      realized: list of dicts sorted by num_ua, each with
                requested_fraction, actual_fraction, num_ua, num_aa.
      collisions: list of dicts describing dropped duplicate fractions.
    """
    seen = {}
    order_seen = []
    collisions = []
    for frac in requested_fractions:
        raw = round_half_up(total_agents * frac)
        num_ua = max(1, min(total_agents - 1, raw))
        if num_ua in seen:
            collisions.append({
                "num_ua": num_ua,
                "kept_requested_fraction": seen[num_ua]["requested_fraction"],
                "dropped_requested_fraction": frac,
            })
            continue
        entry = {
            "requested_fraction": frac,
            "actual_fraction": num_ua / total_agents,
            "num_ua": num_ua,
            "num_aa": total_agents - num_ua,
        }
        seen[num_ua] = entry
        order_seen.append(num_ua)
    realized = sorted((seen[k] for k in order_seen), key=lambda e: e["num_ua"])
    return realized, collisions


def assign_matrix_agents(free, total_agents: int, k_values, rng: random.Random):
    """
    Builds ONE paired/nested random configuration for a whole grid+density+seed
    group, valid for every requested num_ua in k_values simultaneously.

    Validity rule actually required by the Java loader/solvers (see
    InstanceBuilder_MovingAI / Agent.isUnassigned / GoalConstraint): every
    agent's START must be mutually distinct, and every agent's final GOAL
    (its own start, for a UA; a separate cell, for an AA) must ALSO be
    mutually distinct -- but an AA's goal is free to coincide with a
    DIFFERENT agent's start. That is much less restrictive than requiring all
    starts and all AA goals to be one single globally-distinct pool (the
    older random_sweep()/make_random_agents() behaviour), and is what makes
    high density (up to ~80%) achievable.

    Nesting: a single random `order` permutation decides UA priority; at
    num_ua=k, agents order[:k] are UA. Growing k only ever ADDS UAs, so AA
    goals are assigned once (largest k, i.e. fewest AA agents, processed
    first) and then frozen and reused for every smaller k where that agent
    remains AA, per the "preserve pre-generated AA goals" requirement.

    Raises GoalAssignmentInfeasible if some AA agent has no legal goal cell
    left (this can happen on very small/dense grids); callers should treat
    this as "skip this configuration", not a crash.
    """
    free_list = list(free)
    starts = rng.sample(free_list, total_agents)
    order = list(range(total_agents))
    rng.shuffle(order)

    goal_by_agent = {}
    used_goals = set()

    for k in sorted(set(k_values), reverse=True):
        ua_ids = set(order[:k])
        forbidden = set(used_goals)
        forbidden.update(starts[i] for i in ua_ids)
        aa_ids = [i for i in range(total_agents) if i not in ua_ids]
        for aid in aa_ids:
            if aid in goal_by_agent:
                continue
            own_start = starts[aid]
            candidates = [c for c in free_list if c != own_start and c not in forbidden]
            if not candidates:
                raise GoalAssignmentInfeasible(
                    f"No valid AA goal cell for agent {aid} at num_ua={k} "
                    f"(total_agents={total_agents}, free_cells={len(free_list)})."
                )
            choice = rng.choice(candidates)
            goal_by_agent[aid] = choice
            used_goals.add(choice)
            forbidden.add(choice)

    return starts, order, goal_by_agent


def build_matrix_agents(starts, order, goal_by_agent, num_ua: int):
    ua_ids = set(order[:num_ua])
    agents = []
    for i in range(len(starts)):
        if i in ua_ids:
            agents.append({"start": starts[i], "goal": starts[i], "ua": True})
        else:
            agents.append({"start": starts[i], "goal": goal_by_agent[i], "ua": False})
    return agents


def append_matrix_manifest(records, *, scen_path, map_path, grid_width, grid_height,
                            free_cells, requested_density, actual_density,
                            total_agents, requested_ua_fraction, actual_ua_fraction,
                            num_ua, num_aa, seed):
    records.append({
        # New matrix-specific schema (Part 1F).
        "instance_name": scen_path.name,
        "scenario_path": str(scen_path),
        "map_name": map_path.name,
        "map_path": str(map_path),
        "experiment_type": "matrix",
        "grid_width": grid_width,
        "grid_height": grid_height,
        "grid_size": f"{grid_width}x{grid_height}",
        "free_cells": free_cells,
        "requested_agent_density": requested_density,
        "actual_agent_density": actual_density,
        "total_agents": total_agents,
        "requested_ua_fraction": requested_ua_fraction,
        "actual_ua_fraction": actual_ua_fraction,
        "num_ua": num_ua,
        "num_aa": num_aa,
        "seed": seed,
        "expected_mua": "",
        "known_solvable": "",
        # Backward-compatible aliases so the existing single-manifest merge
        # path in analyze_mua_results.py still populates sensibly.
        "instance_family": "matrix",
        "family_parameter": num_ua,
        "agent_density": actual_density,
    })


MATRIX_MANIFEST_FIELDS = [
    "instance_name", "scenario_path", "map_name", "map_path",
    "experiment_type", "grid_width", "grid_height", "grid_size",
    "free_cells", "requested_agent_density", "actual_agent_density",
    "total_agents", "requested_ua_fraction", "actual_ua_fraction",
    "num_ua", "num_aa", "seed", "expected_mua", "known_solvable",
    "instance_family", "family_parameter", "agent_density",
]


def matrix_sweep(args):
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)

    manifest_records = []
    skipped_records = []
    collision_records = []
    run_plan_records = []

    grid_tokens = args.grid_sizes
    density_values = args.density_values
    ua_fractions = args.ua_fractions
    num_seeds = args.seeds

    for token in grid_tokens:
        width, height = parse_grid_size_token(token)
        grid = generate_open_grid(width, height)
        free = [(x, y) for y in range(height) for x in range(width)]
        free_cells = len(free)
        map_stem = f"grid-{width}-{height}"
        grid_size_label = f"{width}x{height}"

        for requested_density in density_values:
            total_agents, actual_density = realize_total_agents(free_cells, requested_density)
            density_code = f"{int(round(requested_density * 100)):03d}"
            group_dir = out / f"grid-{grid_size_label}" / f"density-{density_code}"

            if total_agents < 2:
                skipped_records.append({
                    "grid_size": grid_size_label,
                    "requested_density": requested_density,
                    "seed": "ALL",
                    "requested_ua_fraction": "ALL",
                    "reason": (
                        f"total_agents={total_agents} < 2 for free_cells={free_cells}; "
                        "cannot realize any UA/AA split"
                    ),
                })
                continue

            realized_fractions, collisions = realize_ua_fractions(total_agents, ua_fractions)
            for c in collisions:
                collision_records.append({
                    "grid_size": grid_size_label,
                    "requested_density": requested_density,
                    "total_agents": total_agents,
                    "num_ua": c["num_ua"],
                    "kept_requested_ua_fraction": c["kept_requested_fraction"],
                    "dropped_requested_ua_fraction": c["dropped_requested_fraction"],
                })
                print(
                    f"[matrix] WARNING: grid {grid_size_label} density {requested_density} "
                    f"(total_agents={total_agents}): requested UA fraction "
                    f"{c['dropped_requested_fraction']} rounds to the same num_ua="
                    f"{c['num_ua']} as fraction {c['kept_requested_fraction']}; "
                    "dropping the duplicate experimental point."
                )

            k_values = [e["num_ua"] for e in realized_fractions]
            map_path = group_dir / f"{map_stem}.map"
            map_written = False
            successful_seeds = 0
            scenarios_written = 0

            for seed in range(1, num_seeds + 1):
                rng = random.Random(seed)
                try:
                    starts, order, goal_by_agent = assign_matrix_agents(
                        free, total_agents, k_values, rng
                    )
                except GoalAssignmentInfeasible as e:
                    skipped_records.append({
                        "grid_size": grid_size_label,
                        "requested_density": requested_density,
                        "seed": seed,
                        "requested_ua_fraction": "ALL",
                        "reason": str(e),
                    })
                    continue

                if not map_written:
                    write_map(map_path, grid)
                    map_written = True

                successful_seeds += 1
                for entry in realized_fractions:
                    num_ua = entry["num_ua"]
                    num_aa = entry["num_aa"]
                    agents = build_matrix_agents(starts, order, goal_by_agent, num_ua)
                    scen_name = (
                        f"{map_stem}-random-matrix-d{density_code}-n{total_agents}-"
                        f"ua{num_ua}-aa{num_aa}-seed-{seed:03d}.scen"
                    )
                    scen_path = group_dir / scen_name
                    write_scen(scen_path, map_path.name, width, height, agents)
                    scenarios_written += 1

                    append_matrix_manifest(
                        manifest_records,
                        scen_path=scen_path,
                        map_path=map_path,
                        grid_width=width,
                        grid_height=height,
                        free_cells=free_cells,
                        requested_density=requested_density,
                        actual_density=actual_density,
                        total_agents=total_agents,
                        requested_ua_fraction=entry["requested_fraction"],
                        actual_ua_fraction=entry["actual_fraction"],
                        num_ua=num_ua,
                        num_aa=num_aa,
                        seed=seed,
                    )

            if successful_seeds > 0:
                run_plan_records.append({
                    "grid_size": grid_size_label,
                    "grid_width": width,
                    "grid_height": height,
                    "free_cells": free_cells,
                    "requested_density": requested_density,
                    "actual_density": actual_density,
                    "total_agents": total_agents,
                    "input_directory": str(group_dir),
                    "map_path": str(map_path),
                    "number_of_scenarios": scenarios_written,
                    "number_of_ratio_points": len(realized_fractions),
                    "number_of_seeds": successful_seeds,
                    "requested_seeds": num_seeds,
                })

    return manifest_records, skipped_records, collision_records, run_plan_records


def save_matrix_outputs(output, manifest_records, skipped_records, collision_records, run_plan_records):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)

    manifest_path = out / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=MATRIX_MANIFEST_FIELDS)
        w.writeheader()
        w.writerows(manifest_records)

    run_plan_path = out / "run_plan.csv"
    run_plan_fields = [
        "grid_size", "grid_width", "grid_height", "free_cells",
        "requested_density", "actual_density", "total_agents",
        "input_directory", "map_path", "number_of_scenarios",
        "number_of_ratio_points", "number_of_seeds", "requested_seeds",
    ]
    with run_plan_path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=run_plan_fields)
        w.writeheader()
        w.writerows(run_plan_records)

    skipped_path = out / "skipped_configurations.csv"
    skipped_fields = ["grid_size", "requested_density", "seed", "requested_ua_fraction", "reason"]
    with skipped_path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=skipped_fields)
        w.writeheader()
        w.writerows(skipped_records)

    collisions_path = out / "ua_fraction_collisions.csv"
    collision_fields = [
        "grid_size", "requested_density", "total_agents", "num_ua",
        "kept_requested_ua_fraction", "dropped_requested_ua_fraction",
    ]
    with collisions_path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=collision_fields)
        w.writeheader()
        w.writerows(collision_records)

    print(f"\n[matrix] Generated {len(manifest_records)} scenarios across {len(run_plan_records)} grid+density groups.")
    print(f"[matrix] Manifest:   {manifest_path}")
    print(f"[matrix] Run plan:   {run_plan_path}")
    print(f"[matrix] Skipped:    {skipped_path} ({len(skipped_records)} rows)")
    print(f"[matrix] Collisions: {collisions_path} ({len(collision_records)} rows)")

    if run_plan_records:
        print("\n[matrix] Example run-plan rows -> Java/GenericRunManager arguments:")
        solver_list = (
            "CBSStarMUA,AStarODMUAZero,AStarODMUAMax,"
            "AStarODMUAIH,AStarODMUAMH"
        )
        for row in run_plan_records[:3]:
            print(
                f"  grid {row['grid_size']} density {row['requested_density']} "
                f"(total_agents={row['total_agents']}):"
            )
            print(
                f"    java -jar <jar> -iDir \"{row['input_directory']}\" "
                f"-a {row['total_agents']} -s {solver_list} "
                f"-t 60000 -resDir \"<results_dir>\" -resPref \"grid{row['grid_size']}_d{row['requested_density']}\""
            )


def append_random_obstacle_manifest(records, *, scen_path, map_path, seed, grid_size,
                                    obstacle_density, num_aa, num_ua, free_cells):
    total_agents = num_aa + num_ua
    records.append({
        "instance_name": scen_path.name,
        "scenario_path": str(scen_path),
        "map_name": map_path.name,
        "map_path": str(map_path),
        "seed": seed,
        "grid_size": grid_size,
        "obstacle_density": obstacle_density,
        "num_aa": num_aa,
        "num_ua": num_ua,
        "total_agents": total_agents,
        "free_cells": free_cells,
        "agent_density": total_agents / free_cells if free_cells else "",
    })


RANDOM_OBSTACLE_MANIFEST_FIELDS = [
    "instance_name", "scenario_path", "map_name", "map_path",
    "seed", "grid_size", "obstacle_density",
    "num_aa", "num_ua", "total_agents", "free_cells", "agent_density",
]


def random_obstacle_screen(args, records):
    """
    Reproducible random-obstacle screening dataset (mode: random_obstacle_screen).

    Natural/unfiltered random generation: obstacle cells and agent start/goal
    cells are both drawn by plain uniform sampling (generate_random_obstacle_grid
    / make_random_agents) - no deliberate steering of UAs towards or away from
    bottlenecks, and no pre-filtering by solvability or MUA. The only acceptance
    criterion applied is the hard structural requirement that the map's free-cell
    graph be a single connected component (see generate_random_obstacle_grid).
    One map is generated per (grid size, obstacle density, seed) and shared
    across every requested UA count, since the obstacle layout itself does not
    depend on how many agents are later placed on it.
    """
    out = Path(args.output)
    num_aa = args.aa
    ua_values = sorted(set(args.ua_values))

    for token in args.grid_sizes:
        width, height = parse_grid_size_token(token)
        grid_size_label = f"{width}x{height}"

        for density in args.obstacle_density_values:
            density_code = f"{int(round(density * 100)):03d}"
            group_dir = out / f"grid-{grid_size_label}" / f"obstacle-{density_code}"

            for seed in range(1, args.seeds + 1):
                grid_rng = random.Random(seed)
                grid, free = generate_random_obstacle_grid(width, height, density, grid_rng)
                map_stem = f"grid-{width}-{height}-obstacle-{density_code}-seed-{seed:03d}"
                map_path = group_dir / f"{map_stem}.map"
                write_map(map_path, grid)

                for num_ua in ua_values:
                    agents = make_random_agents(free, num_aa, num_ua, seed)
                    scen_path = group_dir / (
                        f"{map_stem}-random-screen-aa{num_aa}-ua{num_ua}-seed-{seed:03d}.scen"
                    )
                    write_scen(scen_path, map_path.name, width, height, agents)
                    append_random_obstacle_manifest(
                        records, scen_path=scen_path, map_path=map_path, seed=seed,
                        grid_size=grid_size_label, obstacle_density=density,
                        num_aa=num_aa, num_ua=num_ua, free_cells=len(free),
                    )
    return records


def save_random_obstacle_manifest(output, records):
    path = Path(output) / "manifest.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=RANDOM_OBSTACLE_MANIFEST_FIELDS)
        w.writeheader()
        w.writerows(records)
    print(f"Generated {len(records)} random-obstacle screening scenarios.")
    print(f"Manifest: {path}")


def validate_random_obstacle_screen(output):
    """
    Post-generation structural validation (does not launch Java / solve
    anything): re-reads the manifest and, for every row, re-derives the free
    cells from the .map file and re-parses the .scen file, then checks:
      - the referenced .map/.scen files exist,
      - the map's free-cell graph is a single connected component,
      - every agent start cell is a free cell and all starts are mutually distinct,
      - AA rows (start != goal) have reachable start/goal ignoring other agents
        (always true once the whole map is one connected component, but checked
        explicitly rather than assumed),
      - UA rows have start == goal.
    Returns (total_instances, by_obstacle_density, by_num_ua, errors).
    """
    out = Path(output)
    manifest_path = out / "manifest.csv"
    with manifest_path.open("r", newline="", encoding="utf-8-sig") as f:
        rows = list(csv.DictReader(f))

    by_density: "dict[str, int]" = {}
    by_num_ua: "dict[str, int]" = {}
    errors = []

    for row in rows:
        density_key = row["obstacle_density"]
        by_density[density_key] = by_density.get(density_key, 0) + 1
        ua_key = row["num_ua"]
        by_num_ua[ua_key] = by_num_ua.get(ua_key, 0) + 1

        instance = row["instance_name"]
        map_path = Path(row["map_path"])
        scen_path = Path(row["scenario_path"])

        if not map_path.is_file():
            errors.append(f"{instance}: missing map file {map_path}")
            continue
        if not scen_path.is_file():
            errors.append(f"{instance}: missing scenario file {scen_path}")
            continue

        _, free, width, height = load_map(map_path)
        if not free_cells_connected(free):
            errors.append(f"{instance}: map free-cell graph is not connected")
        free_set = set(free)

        scen_lines = scen_path.read_text().splitlines()[1:]
        starts = []
        for line in scen_lines:
            if not line.strip():
                continue
            parts = line.split("\t")
            sx, sy, gx, gy = int(parts[4]), int(parts[5]), int(parts[6]), int(parts[7])
            start, goal = (sx, sy), (gx, gy)
            starts.append(start)
            if start not in free_set:
                errors.append(f"{instance}: agent start {start} is not a free cell")
            if start == goal:
                continue  # UA: start == goal, as required
            if goal not in free_set:
                errors.append(f"{instance}: AA goal {goal} is not a free cell")
            # AA: start != goal and must be mutually reachable ignoring other agents -
            # trivially true on a fully-connected map, verified explicitly anyway.
            if start in free_set and goal in free_set and not _reachable(free_set, start, goal):
                errors.append(f"{instance}: AA start {start} cannot reach goal {goal} ignoring other agents")

        if len(starts) != len(set(starts)):
            errors.append(f"{instance}: agent start cells are not all distinct")

        expected_num_aa = int(row["num_aa"])
        expected_num_ua = int(row["num_ua"])
        actual_num_ua = sum(1 for line in scen_lines if line.strip() and
                            (lambda p: (int(p[4]), int(p[5])) == (int(p[6]), int(p[7])))(line.split("\t")))
        actual_num_aa = len(starts) - actual_num_ua
        if actual_num_aa != expected_num_aa or actual_num_ua != expected_num_ua:
            errors.append(
                f"{instance}: manifest says aa={expected_num_aa}/ua={expected_num_ua}, "
                f"scenario file has aa={actual_num_aa}/ua={actual_num_ua}"
            )

    return len(rows), by_density, by_num_ua, errors


def _reachable(free_set, start, goal) -> bool:
    if start == goal:
        return True
    visited = {start}
    stack = [start]
    while stack:
        x, y = stack.pop()
        if (x, y) == goal:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy)
            if n in free_set and n not in visited:
                visited.add(n)
                stack.append(n)
    return goal in visited


# --------------------------------------------------------------------------------------
# random_plan mode: ordinary random instances for every row of an experiment plan CSV
# (e.g. Experiments/MUA/table_5x5/experiment_plan_5x5.csv).
# --------------------------------------------------------------------------------------

RANDOM_PLAN_MANIFEST_FIELDS = [
    "instance_name", "scenario_path", "map_name", "map_path",
    "experiment_type", "instance_family", "grid_size", "grid_width", "grid_height",
    "seed", "rng_key", "num_aa", "num_ua", "total_agents", "free_cells", "agent_density",
    "expected_mua", "known_solvable", "plan_path",
]


def find_repo_root(start: Path) -> Path:
    for p in [start, *start.parents]:
        if (p / "pom.xml").is_file():
            return p
    raise SystemExit(f"Could not find the repository root (pom.xml) above {start}")


def repo_relative(path: Path, repo_root: Path) -> str:
    """Repository-relative, forward-slash path (portable between Windows and Linux)."""
    try:
        return path.resolve().relative_to(repo_root).as_posix()
    except ValueError:
        raise SystemExit(f"{path} is outside the repository {repo_root}; manifest paths must be repo-relative.")


def load_experiment_plan(plan_path: Path) -> List[dict]:
    with plan_path.open("r", encoding="utf-8-sig", newline="") as f:
        rows = [dict(r) for r in csv.DictReader(f)]
    required = {"grid_size", "num_aa", "total_agents", "num_ua", "expected_random_instances"}
    if not rows or not required <= set(rows[0]):
        raise SystemExit(f"{plan_path} must have columns {sorted(required)}")
    plan, seen = [], set()
    for r in rows:
        cfg = {
            "grid_size": r["grid_size"].strip(),
            "num_aa": int(r["num_aa"]),
            "total_agents": int(r["total_agents"]),
            "num_ua": int(r["num_ua"]),
            "instances": int(r["expected_random_instances"]),
        }
        key = (cfg["grid_size"], cfg["num_aa"], cfg["total_agents"])
        if key in seen:
            raise SystemExit(f"duplicated plan configuration {key}")
        seen.add(key)
        if cfg["num_ua"] != cfg["total_agents"] - cfg["num_aa"] or cfg["num_aa"] < 1 or cfg["num_ua"] < 0:
            raise SystemExit(f"inconsistent plan row {r}")
        plan.append(cfg)
    return plan


def make_plan_agents(free, num_aa: int, num_ua: int, rng: random.Random):
    """
    Ordinary random MAPFUA agents for random_plan mode:
      - all starts are distinct;
      - UAs: goal == start;
      - AAs: goals distinct among AAs and different from the AA's own start.
    An AA goal MAY coincide with another agent's start (unlike make_random_agents), so
    dense configurations only need total_agents <= free cells and num_aa < free cells.
    AA goals are drawn by rejection sampling, i.e. uniformly over all valid goal
    assignments. No attempt is made to force MUA > 0 or solvability.
    """
    total = num_aa + num_ua
    if total > len(free) or num_aa >= len(free):
        raise ValueError(f"{num_aa} AA + {num_ua} UA do not fit on {len(free)} free cells")
    starts = rng.sample(free, total)
    while True:
        goals = rng.sample(free, num_aa)
        if all(goals[i] != starts[i] for i in range(num_aa)):
            break
    agents = [{"start": starts[i], "goal": goals[i], "ua": False} for i in range(num_aa)]
    agents += [{"start": s, "goal": s, "ua": True} for s in starts[num_aa:]]
    return agents


def random_plan(args):
    repo_root = find_repo_root(Path(__file__).resolve().parent)
    plan_path = Path(args.plan).resolve()
    plan = load_experiment_plan(plan_path)
    out = Path(args.output).resolve()
    inst_dir = out / "instances"
    if inst_dir.exists() and any(inst_dir.iterdir()):
        if not args.overwrite:
            raise SystemExit(f"{inst_dir} is not empty; pass --overwrite to regenerate it.")
        shutil.rmtree(inst_dir)
    inst_dir.mkdir(parents=True, exist_ok=True)

    # One map per grid size: an explicit --map (copied next to the scenarios) or an open grid.
    maps = {}
    for grid_size in sorted({c["grid_size"] for c in plan}):
        w, h = parse_grid_size_token(grid_size)
        if args.map:
            src = Path(args.map)
            grid, free, mw, mh = load_map(src)
            if (mw, mh) != (w, h):
                raise SystemExit(f"--map {src} is {mw}x{mh} but the plan asks for {grid_size}")
            map_path = inst_dir / src.name
            shutil.copyfile(src, map_path)
        else:
            grid = generate_open_grid(w, h)
            map_path = inst_dir / f"grid-{w}x{h}-open.map"
            write_map(map_path, grid)
            _, free, _, _ = load_map(map_path)
        maps[grid_size] = (map_path, free, w, h)

    records = []
    for cfg in plan:
        map_path, free, w, h = maps[cfg["grid_size"]]
        for seed in range(1, cfg["instances"] + 1):
            rng_key = f"random_plan|{cfg['grid_size']}|aa{cfg['num_aa']}|total{cfg['total_agents']}|seed{seed}"
            agents = make_plan_agents(free, cfg["num_aa"], cfg["num_ua"], random.Random(rng_key))
            # Name must start with "<map stem>-random": InstanceBuilder_MovingAI pairs a .scen
            # with the .map whose stem precedes "-random" in the scenario path.
            scen_path = inst_dir / (f"{map_path.stem}-random-aa{cfg['num_aa']}"
                                    f"-total{cfg['total_agents']}-seed{seed:02d}.scen")
            write_scen(scen_path, map_path.name, w, h, agents)
            records.append({
                "instance_name": scen_path.name,
                "scenario_path": repo_relative(scen_path, repo_root),
                "map_name": map_path.name,
                "map_path": repo_relative(map_path, repo_root),
                "experiment_type": "random_plan",
                "instance_family": "random",
                "grid_size": cfg["grid_size"],
                "grid_width": w,
                "grid_height": h,
                "seed": seed,
                "rng_key": rng_key,
                "num_aa": cfg["num_aa"],
                "num_ua": cfg["num_ua"],
                "total_agents": cfg["total_agents"],
                "free_cells": len(free),
                "agent_density": round(cfg["total_agents"] / len(free), 6),
                "expected_mua": "",
                "known_solvable": "",
                "plan_path": repo_relative(plan_path, repo_root),
            })

    manifest = out / "manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=RANDOM_PLAN_MANIFEST_FIELDS)
        w.writeheader()
        w.writerows(records)
    print(f"Generated {len(records)} scenarios for {len(plan)} configurations.")
    print(f"Manifest: {manifest}")
    return manifest, plan, repo_root


def read_scen_agents(path: Path):
    agents = []
    lines = path.read_text().splitlines()
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = line.split("\t")
        agents.append(((int(parts[4]), int(parts[5])), (int(parts[6]), int(parts[7]))))
    return lines[0], agents


def validate_random_plan(manifest: Path, plan: List[dict], repo_root: Path) -> List[str]:
    """Re-reads every generated file from disk and checks the random_plan invariants."""
    with manifest.open("r", encoding="utf-8-sig", newline="") as f:
        rows = [dict(r) for r in csv.DictReader(f)]
    errors = []
    expected = {(c["grid_size"], c["num_aa"], c["total_agents"]): c for c in plan}
    seeds_by_cfg = {}
    for r in rows:
        cfg_key = (r["grid_size"], int(r["num_aa"]), int(r["total_agents"]))
        seeds_by_cfg.setdefault(cfg_key, []).append(int(r["seed"]))
        name = r["instance_name"]
        num_aa, num_ua, total = int(r["num_aa"]), int(r["num_ua"]), int(r["total_agents"])
        w, h = int(r["grid_width"]), int(r["grid_height"])
        if num_ua != total - num_aa:
            errors.append(f"{name}: num_ua != total_agents - num_aa")
        for col in ("scenario_path", "map_path"):
            if "\\" in r[col] or Path(r[col]).is_absolute() or ":" in r[col]:
                errors.append(f"{name}: {col} is not a repo-relative forward-slash path: {r[col]}")
        scen = repo_root / r["scenario_path"]
        map_file = repo_root / r["map_path"]
        if not scen.is_file() or not map_file.is_file():
            errors.append(f"{name}: scenario or map file missing")
            continue
        if scen.parent != map_file.parent:
            errors.append(f"{name}: map is not in the scenario directory")
        if not scen.stem.split("-random")[0].endswith(map_file.stem):
            errors.append(f"{name}: InstanceBuilder_MovingAI would not pair it with {map_file.name}")
        _, free, mw, mh = load_map(map_file)
        if (mw, mh) != (w, h) or int(r["free_cells"]) != len(free):
            errors.append(f"{name}: map size/free cells mismatch")
        _, agents = read_scen_agents(scen)
        if len(agents) != total:
            errors.append(f"{name}: {len(agents)} agent rows, expected {total}")
        starts = [s for s, _ in agents]
        if len(set(starts)) != len(starts):
            errors.append(f"{name}: duplicated start cells")
        aa, ua = agents[:num_aa], agents[num_aa:]
        if any(s == g for s, g in aa):
            errors.append(f"{name}: an AA has goal == start")
        if any(s != g for s, g in ua):
            errors.append(f"{name}: a UA has goal != start")
        if len({g for _, g in aa}) != len(aa):
            errors.append(f"{name}: duplicated AA goals")
        free_set = set(free)
        if any(not (0 <= x < w and 0 <= y < h) or (x, y) not in free_set for s, g in agents for (x, y) in (s, g)):
            errors.append(f"{name}: coordinate outside the map / on an obstacle")
    if len(seeds_by_cfg) != len(plan) or set(seeds_by_cfg) != set(expected):
        errors.append(f"configurations: generated {len(seeds_by_cfg)}, plan has {len(plan)}")
    for key, c in expected.items():
        if sorted(seeds_by_cfg.get(key, [])) != list(range(1, c["instances"] + 1)):
            errors.append(f"{key}: seeds are not exactly 1..{c['instances']}")
    if len({r["instance_name"] for r in rows}) != len(rows):
        errors.append("instance names are not unique")
    if len({r["scenario_path"] for r in rows}) != len(rows):
        errors.append("scenario paths are not unique")
    expected_total = sum(c["instances"] for c in plan)
    if len(rows) != expected_total:
        errors.append(f"manifest has {len(rows)} rows, plan expects {expected_total}")

    print(f"\nCONFIGURATIONS = {len(seeds_by_cfg)}")
    print(f"SCENARIOS      = {len(rows)} (expected {expected_total})")
    print("COUNTS BY num_aa: total_agents=count")
    for aa in sorted({k[1] for k in seeds_by_cfg}):
        cells = [f"{k[2]}={len(seeds_by_cfg[k])}" for k in sorted(seeds_by_cfg) if k[1] == aa]
        print(f"  aa={aa:<2} " + " ".join(cells))
    print(f"STRUCTURAL_ERRORS = {len(errors)}")
    for e in errors[:30]:
        print(f"  - {e}")
    return errors


def save_manifest(output, records):
    path = Path(output) / "manifest.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "instance_name", "scenario_path", "map_name", "map_path",
        "instance_family", "family_parameter", "seed",
        "num_aa", "num_ua", "total_agents", "free_cells",
        "agent_density", "expected_mua", "known_solvable"
    ]
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(records)
    print(f"Generated {len(records)} scenarios.")
    print(f"Manifest: {path}")


def build_parser():
    p = argparse.ArgumentParser(description="Generate MAPFUA-MUA experiment instances.")
    p.add_argument(
        "mode",
        choices=[
            "ua_scale", "aa_scale", "density", "ratio",
            "corridor", "k_blockers", "decoy", "decoy_fixed", "unsolvable",
            "all_constructed", "matrix", "positive_mua", "random_obstacle_screen",
            "random_plan"
        ],
    )
    p.add_argument("--output", default="generated_mua_instances")
    p.add_argument("--map", help="Existing MovingAI .map file for random modes "
                                 "(random_plan: optional; default is an open grid per plan grid_size)")
    p.add_argument("--plan", help="random_plan mode: experiment plan CSV "
                                  "(grid_size,num_aa,total_agents,num_ua,expected_random_instances)")
    p.add_argument("--overwrite", action="store_true",
                   help="random_plan mode: replace an existing non-empty <output>/instances directory")
    p.add_argument("--seeds", type=int, default=30)

    p.add_argument("--aa", type=int, default=4, help="Fixed AA for ua_scale")
    p.add_argument("--ua", type=int, default=8, help="Fixed UA for aa_scale")
    p.add_argument("--ua-values", type=int, nargs="+", default=[2, 4, 6, 8, 10, 12])
    p.add_argument("--aa-values", type=int, nargs="+", default=[2, 4, 6, 8, 10])
    p.add_argument("--total-agents", type=int, default=12)
    p.add_argument("--density-values", type=float, nargs="+", default=[0.20, 0.35, 0.50, 0.65])
    p.add_argument("--aa-fraction", type=float, default=0.40)

    p.add_argument("--length-values", type=int, nargs="+", default=[7, 9, 11, 15, 21, 31])
    p.add_argument("--k-values", type=int, nargs="+", default=[1, 2, 3, 4, 5])

    p.add_argument(
        "--grid-sizes", nargs="+", default=["10"],
        help="matrix mode: grid sizes, either integers N (NxN) or WxH tokens like 10x15."
    )
    p.add_argument(
        "--ua-fractions", type=float, nargs="+", default=[0.10, 0.20, 0.30, 0.40, 0.50],
        help="matrix mode: UA fractions of the fixed total-agent population per grid+density group."
    )

    p.add_argument(
        "--positive-num-aa-values", type=int, nargs="+", default=[2, 3],
        help="positive_mua mode: AA counts to generate for every family."
    )
    p.add_argument(
        "--positive-mua-seeds", type=int, default=5,
        help="positive_mua mode: number of semantics-preserving seed variations per configuration."
    )
    p.add_argument(
        "--required-mua-values", type=int, nargs="+", default=[1, 2, 3],
        help="positive_mua required_mua family: expected_mua values (each a distinct required blocker count)."
    )
    p.add_argument(
        "--positive-decoy-values", type=int, nargs="+", default=[0, 2, 4],
        help="positive_mua decoy_ua family: extra_decoy_ua values, with expected_mua fixed at 1."
    )
    p.add_argument(
        "--mixed-mua-values", type=int, nargs="+", default=[2, 3],
        help="positive_mua mixed family: expected_mua values."
    )
    p.add_argument(
        "--mixed-decoy-values", type=int, nargs="+", default=[2, 4],
        help="positive_mua mixed family: extra_decoy_ua values."
    )

    p.add_argument(
        "--obstacle-density-values", type=float, nargs="+", default=[0.10, 0.20, 0.30],
        help="random_obstacle_screen mode: fraction of cells that are obstacles."
    )
    return p


def main():
    args = build_parser().parse_args()

    if args.mode == "matrix":
        manifest_records, skipped_records, collision_records, run_plan_records = matrix_sweep(args)
        save_matrix_outputs(args.output, manifest_records, skipped_records, collision_records, run_plan_records)
        return

    if args.mode == "positive_mua":
        records = []
        positive_mua_sweep(args, records)
        save_positive_mua_manifest(args.output, records)
        return

    if args.mode == "random_plan":
        if not args.plan:
            raise SystemExit("--plan is required for random_plan mode.")
        manifest, plan, repo_root = random_plan(args)
        errors = validate_random_plan(manifest, plan, repo_root)
        if errors:
            raise SystemExit(1)
        return

    if args.mode == "random_obstacle_screen":
        records = []
        random_obstacle_screen(args, records)
        save_random_obstacle_manifest(args.output, records)

        total, by_density, by_num_ua, errors = validate_random_obstacle_screen(args.output)
        print(f"\nTOTAL_INSTANCES = {total}")
        print("BY_OBSTACLE_DENSITY:")
        for k in sorted(by_density, key=float):
            print(f"  {k}: {by_density[k]}")
        print("BY_NUM_UA:")
        for k in sorted(by_num_ua, key=int):
            print(f"  {k}: {by_num_ua[k]}")
        print(f"STRUCTURAL_ERRORS = {len(errors)}")
        for e in errors[:20]:
            print(f"  - {e}")
        return

    records = []

    if args.mode in {"ua_scale", "aa_scale", "density", "ratio"}:
        if not args.map:
            raise SystemExit("--map is required for random modes.")
        random_sweep(args, args.mode, records)
    elif args.mode == "corridor":
        constructed_corridor(args, records)
    elif args.mode == "k_blockers":
        constructed_k_blockers(args, records)
    elif args.mode == "decoy":
        constructed_decoy(args, records)
    elif args.mode == "decoy_fixed":
        constructed_decoy_fixed(args, records)
    elif args.mode == "unsolvable":
        constructed_unsolvable(args, records)
    elif args.mode == "all_constructed":
        constructed_corridor(args, records)
        constructed_k_blockers(args, records)
        constructed_decoy(args, records)
        constructed_unsolvable(args, records)

    save_manifest(args.output, records)


if __name__ == "__main__":
    main()
