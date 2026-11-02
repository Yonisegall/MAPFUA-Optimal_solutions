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
joint timestep's movement/history counters, and the stored `g` of a partial
node is its base full state's `g`; completed full joint timesteps add the step
cost, so SST is not charged twice.

For MUA, the search uses `effectiveG` (OPEN order, `bestG` dominance, stale
check): a partial node is charged immediately for the UA first moves already
chosen in its prefix (`g + stepNewUA`). Those charges are irrevocable, and the
heuristic already treats those UAs as moved, so without this every intra-step
UA-move combination sat on the parent's f plateau. This is standard OD cost
decomposition; a completed child's `g` (`base.g + stepNewUA`) is unchanged.
Fuel and SST use the stored `g`.

`MuaSearchState` stores current positions, OD phase, partial next positions,
the moved-UA `BitSet`, and the transient `visitedTargets` `BitSet`. The
duplicate key includes these values and omits absolute time. The direct A*OD
solver has no future time-indexed constraints, so dropping time prevents
zero-cost wait cycles while preserving objective-relevant and feasibility
history. UAs have no terminal target requirement and are not forced back to
their source.

`MapfuaCaptureHeuristic` implements the source paper's capture relaxation.
Unmoved original UAs are capturable obstacles; capturing a UA removes it from
the relaxed problem and is charged once. Already moved UAs were paid for in `g`
and are relaxed away. Only active AAs (target not yet visited) are considered.
`cap(ai)` is the minimum number of distinct unmoved UAs that `ai` must capture
on its way to its target.

Pure MUA heuristics (exactly the source paper's three, plus uninformed A*):

- `ZERO`: returns 0.
- `MAX`: `max_i cap(ai)`, each active AA evaluated independently.
- `INCREASING` (IH): first check whether every active AA can reach its target
  without removing any UA (h=0); otherwise try every single unmoved UA, then
  every pair, triple, ... in deterministic lexicographic order. Returns the
  first k for which some k-subset lets every active AA independently reach its
  target.
- `MERGE` (MH): all active AAs merged into one meta-agent; relaxed joint search
  (vertex/swap legal among AAs, reach-goal-once) minimizing distinct captures.

On tested states `MAX <= IH <= MH <= exact remaining MUA`.

Pure Fuel heuristics:

- `ZERO`: returns 0.
- `MAX` and `INCREASING`: return the RDC independent obstacle-aware distance
  lower bound over active AAs. This ignores possible UA activation fuel, so it
  cannot overestimate real remaining Fuel.
- `MERGE`: relaxed joint search charging AA moves plus one movement for every
  newly captured unmoved UA. Old captures are not charged again.

Pure SST heuristics:

- `ZERO`: returns 0.
- `MAX` and `INCREASING`: return the independent RDC shortest-service-time
  lower bound over active AAs. Capture cardinality is not used for SST.
- `MERGE`: relaxed joint search over active AAs minimizing remaining SST. UAs
  may be captured/removed for free because UA movement is not part of SST.

`ReachabilityDistanceCache` is keyed by target and exact `U_static` mask. It
builds reverse BFS distances treating unmoved UAs as hard obstacles; unreachable
queries fall back to the coordinate distance as an admissible lower bound. A
new key is used whenever a UA first moves and leaves `U_static`.

Value-preserving speed-ups (checked against verbatim reference copies of the original
h1/h4/h3 code by `MapfuaHeuristicPaperComplianceTest`):

- Every heuristic value is memoized in a bounded LRU cache keyed by exactly what it reads
  (active AAs, their positions, `movedUAs`). Most queries repeat: OD intermediate nodes that only
  move UAs leave the AA positions unchanged.
- MH orders equal-capture nodes by the remaining distance of unserved AAs and stops as soon as a
  zero-capture step generates a goal. It is still uniform-cost search on captures, so the
  returned minimum is unchanged.

Extra report fields: `MAPFUA_MA_heuristic_cache_hits`, `MAPFUA_MA_heuristic_time_ms`,
`MAPFUA_MA_root_h`.

Lazy heuristic evaluation is available through `AStarODMuaBuilder`. It inherits
the parent value only when `movedUAs`, `visitedTargets`, and all active AA
positions are unchanged; otherwise it recomputes eagerly.

Factory/CLI names:

- `AStarODMUAZero`, `AStarODMUAMax`, `AStarODMUAIH`, `AStarODMUAMH`
- `AStarODFuelZero`, `AStarODFuelMax`, `AStarODFuelIH`, `AStarODFuelMH`
- `AStarODSSTZero`, `AStarODSSTMax`, `AStarODSSTIH`, `AStarODSSTMH`

Former names (`H1Max`, `H2EnhancedMax`, `H3Joint`, `H4RandomPick`) are no
longer registered. H1/H3/H4 were the same computations as MAX/MH/IH; H2
(thesis "Enhanced Max") is not a source-paper heuristic and was removed.

Known limitation: for Fuel and SST, MAX and IH fall back to the same
independent-distance bound, because capture cardinality is not itself an
admissible estimate of those objectives.
