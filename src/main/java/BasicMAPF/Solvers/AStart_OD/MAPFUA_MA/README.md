# MAPFUA-MA A*OD

This package implements Bar Dolev's Chapter 4 MAPFUA-MA A*OD structure using
this repository's transient MAPFUA conventions. The search engine is shared by
all registered configurations; `MapfuaObjective` selects one pure objective for
`g`, `h`, `f`, duplicate dominance, metrics, and reported cost.

Objectives:

- `MUA`: first non-wait move of an original `Agent.isUnassigned` agent costs 1;
  all later moves by that UA, AA moves, and waits cost 0.
- `FUEL`: every non-wait move by any AA or UA costs 1; waits cost 0.
- `SST`: every completed timestep costs 1 for each AA that had not visited its
  target before that timestep. UAs cost 0.

No objective is used as a tie-breaker for another. OPEN orders by scalar
`f = g + h` and then deterministic node id. OD partial nodes accumulate one
joint timestep's movement/history counters, but only completed full joint
timesteps add to `g`, so SST is not charged twice.

`MuaSearchState` stores current positions, OD phase, partial next positions,
the moved-UA `BitSet`, and the transient `visitedTargets` `BitSet`. The
duplicate key includes these values and omits absolute time. The direct A*OD
solver has no future time-indexed constraints, so dropping time prevents
zero-cost wait cycles while preserving objective-relevant and feasibility
history. UAs have no terminal target requirement and are not forced back to
their source.

`MapfuaCaptureHeuristic` implements MAPFUA-CA. Unmoved original UAs are
capturable obstacles; capturing a UA removes it from the relaxed problem and is
charged once. Already moved UAs are relaxed away.

Pure MUA heuristics:

- `ZERO`: returns 0.
- `MAX`: maximum per-AA minimum capture cardinality.
- `ENHANCED_MAX`: Algorithm 4.2 helpful-set test, returning K or K+1 captures.
- `JOINT_CAPTURE`: relaxed joint MAPFUA-CA search minimizing distinct captures.
- `RANDOM_PICK`: exhaustive increasing-cardinality helpful subset search.

Pure Fuel heuristics:

- `ZERO`: returns 0.
- `MAX`, `ENHANCED_MAX`, and `RANDOM_PICK`: return the RDC independent
  obstacle-aware distance lower bound over active AAs. This ignores possible UA
  activation fuel unless h3 proves it in the relaxed joint search, so it cannot
  overestimate real remaining Fuel.
- `JOINT_CAPTURE`: relaxed joint MAPFUA-CA search charging AA moves plus one
  movement for every newly captured unmoved UA. Old captures are not charged
  again.

Pure SST heuristics:

- `ZERO`: returns 0.
- `MAX`, `ENHANCED_MAX`, and `RANDOM_PICK`: return the independent RDC
  shortest-service-time lower bound over active AAs. Capture cardinality is not
  used for SST.
- `JOINT_CAPTURE`: relaxed joint search over active AAs minimizing remaining
  SST. UAs may be captured/removed for free because UA movement is not part of
  SST.

`ReachabilityDistanceCache` is keyed by target and exact `U_static` mask. It
builds reverse BFS distances treating unmoved UAs as hard obstacles; unreachable
queries fall back to the coordinate distance as an admissible lower bound. A
new key is used whenever a UA first moves and leaves `U_static`.

Lazy heuristic evaluation is available through `AStarODMuaBuilder`. It inherits
the parent value only when `movedUAs`, `visitedTargets`, and all active AA
positions are unchanged; otherwise it recomputes eagerly.

Factory/CLI names:

- `AStarODMUAZero`, `AStarODMUAH1Max`, `AStarODMUAH2EnhancedMax`,
  `AStarODMUAH3Joint`, `AStarODMUAH4RandomPick`
- `AStarODFuelZero`, `AStarODFuelH1Max`, `AStarODFuelH2EnhancedMax`,
  `AStarODFuelH3Joint`, `AStarODFuelH4RandomPick`
- `AStarODSSTZero`, `AStarODSSTH1Max`, `AStarODSSTH2EnhancedMax`,
  `AStarODSSTH3Joint`, `AStarODSSTH4RandomPick`

Known limitation: h2 and h4 use conservative objective-specific analogues for
Fuel and SST where the original capture-cardinality structure is not itself an
admissible estimate of those objectives.
