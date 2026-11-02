package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.InstanceBuilders.InstanceBuilder_ArbitraryGraph;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.Coordinates.Coordinate_2D;
import BasicMAPF.Instances.Maps.Enum_MapLocationType;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Instances.Maps.MapFactory;
import BasicMAPF.Solvers.CanonicalSolversFactory;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.Metrics;
import org.junit.jupiter.api.Test;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Paper-compliance checks for the Section 7.3 heuristics (MAX, IH, MH) against:
 *  - independent REFERENCE implementations (verbatim semantics of the pre-683bc58 h1/h4/h3 code), so any
 *    performance change to MapfuaCaptureHeuristic must return exactly the same value on every sampled state;
 *  - a brute-force exact remaining-MUA oracle over the full joint state space (admissibility);
 *  - each other (MAX <= IH <= MH on full joint states);
 *  - cross-solver optimality on small random instances (including infeasible ones).
 * States are sampled throughout the search (random walks), including OD intermediate states.
 */
class MapfuaHeuristicPaperComplianceTest {
    private static final long INF = Long.MAX_VALUE;
    private static final List<MuaHeuristicType> INFORMED = List.of(MuaHeuristicType.MAX, MuaHeuristicType.INCREASING, MuaHeuristicType.MERGE);

    // ---------------------------------------------------------------------------------------------
    // Tests
    // ---------------------------------------------------------------------------------------------

    @Test
    void productionHeuristicsEqualReferenceImplementationsOnFullAndOdStates() {
        int compared = 0;
        for (Case c : cases(40, 7)) {
            MuaProblemContext ctx = new MuaProblemContext(c.instance);
            for (Sample s : sampleStates(ctx, new Random(c.seed), 4, 6, true)) {
                MuaSearchState st = s.state;
                assertEquals(refMax(ctx, st), h(ctx, MuaHeuristicType.MAX, st), c.name + " MAX " + s);
                assertEquals(refIH(ctx, st), h(ctx, MuaHeuristicType.INCREASING, st), c.name + " IH " + s);
                assertEquals(refMH(ctx, st), h(ctx, MuaHeuristicType.MERGE, st), c.name + " MH " + s);
                assertEquals(0, h(ctx, MuaHeuristicType.ZERO, st));
                compared++;
            }
        }
        System.out.println("[paper-compliance] reference-equivalence states compared: " + compared);
        assertTrue(compared > 500);
    }

    @Test
    void cachedHeuristicReturnsSameValueOnRepeatedQueries() {
        for (Case c : cases(10, 11)) {
            MuaProblemContext ctx = new MuaProblemContext(c.instance);
            for (MuaHeuristicType type : INFORMED) {
                MapfuaCaptureHeuristic shared = new MapfuaCaptureHeuristic(ctx, MapfuaObjective.MUA, type, false);
                List<Sample> samples = sampleStates(ctx, new Random(c.seed), 3, 5, true);
                for (int pass = 0; pass < 2; pass++) {
                    for (Sample s : samples) {
                        assertEquals(h(ctx, type, s.state), shared.estimate(s.state), c.name + " " + type + " pass " + pass);
                    }
                }
            }
        }
    }

    @Test
    void heuristicsAreAdmissibleOnFullAndOdStatesAndOrderedOnFullStates() {
        int full = 0, partial = 0, partialOrderViolations = 0;
        for (Case c : cases(40, 3)) {
            MuaProblemContext ctx = new MuaProblemContext(c.instance);
            Oracle oracle = new Oracle(ctx);
            for (Sample s : sampleStates(ctx, new Random(c.seed), 3, 5, true)) {
                long exact = s.partial ? oracle.completionCost(s) : oracle.exact(s.state);
                long max = h(ctx, MuaHeuristicType.MAX, s.state);
                long ih = h(ctx, MuaHeuristicType.INCREASING, s.state);
                long mh = h(ctx, MuaHeuristicType.MERGE, s.state);
                String where = c.name + " " + s + " MAX=" + max + " IH=" + ih + " MH=" + mh + " exact=" + (exact == INF ? "INF" : exact);
                for (long v : new long[]{max, ih, mh}) {
                    assertTrue(v >= 0, where);
                    assertTrue(exact == INF || v <= exact, "inadmissible: " + where);
                }
                if (s.partial) {
                    partial++;
                    if (!(max <= ih && ih <= mh)) partialOrderViolations++;
                } else {
                    full++;
                    if (exact != INF) assertTrue(max <= ih && ih <= mh, "dominance: " + where);
                }
            }
        }
        System.out.println("[paper-compliance] admissibility: full=" + full + " odPartial=" + partial
                + " odPartialStatesViolatingMaxLeIhLeMh=" + partialOrderViolations);
        assertTrue(full > 300 && partial > 300);
    }

    /**
     * Documents (does not hide) an OD-intermediate corner case: after a prefix AA has moved into the cell of
     * a UA that has not moved yet in this timestep, IH treats the AA's own cell as blocked (that UA must be
     * removed: IH >= 1), while cap(ai) and MH only charge UAs whose cell is ENTERED by a move, so they ignore
     * it (MH charges it only if the AA waits on or re-enters that cell). All three stay admissible (the UA really is forced to move this step), but MAX <= IH <= MH can fail
     * on such OD intermediate states. Full joint states (the paper's state space) are unaffected.
     */
    @Test
    void odIntermediateStateWithAaOnUnmovedUaCellIsChargedDifferentlyByCapIhMh() {
        MAPF_Instance instance = arbitrary("cbs_star_side_pocket.scene", 3); // L-C-R, C-P, P-Q; AA0 L->R, AA1 R->L, UA at P
        MuaProblemContext ctx = new MuaProblemContext(instance);
        I_Location l = ctx.map.getMapLocation(ctx.agents.get(0).source);
        I_Location p = ctx.map.getMapLocation(ctx.agents.get(2).source);
        I_Location c = l.outgoingEdges().get(0);
        I_Location q = p.outgoingEdges().stream().filter(x -> !x.equals(c)).findFirst().orElseThrow();
        // Full base state: AA0 on C, AA1 on R, UA on P. Joint move: AA0 C->P, AA1 waits, UA P->Q.
        I_Location[] basePos = starts(ctx);
        basePos[0] = c;
        MuaSearchState base = MuaSearchState.full(1, basePos, new BitSet(), rootState(ctx).visitedTargets, 0, 0, null, null, 0);
        I_Location[] next = {p, basePos[1], q};
        Sample od = partialSample(ctx, base, next, 1); // OD prefix = AA0 only: AA0 already on P, UA not moved yet
        Oracle oracle = new Oracle(ctx);
        assertEquals(p, od.state.positions[0]);
        assertTrue(od.state.movedUAs.isEmpty());
        assertEquals(0, h(ctx, MuaHeuristicType.MAX, od.state));
        assertEquals(1, h(ctx, MuaHeuristicType.INCREASING, od.state));
        // MH charges a capture whenever an AA's next position is an unmoved UA cell, including a WAIT on it:
        // here AA0 must wait on P while AA1 passes C, so MH = 1 (it would be 0 if only entering moves counted).
        assertEquals(1, h(ctx, MuaHeuristicType.MERGE, od.state));
        assertEquals(1, oracle.completionCost(od), "the UA on P is forced to move in this step");
    }

    @Test
    void allFourSolversReturnTheOracleOptimumOnSmallRandomInstances() {
        int solvable = 0, infeasible = 0;
        for (Case c : cases(30, 5)) {
            MuaProblemContext ctx = new MuaProblemContext(c.instance);
            long optimum = new Oracle(ctx).exact(rootState(ctx));
            for (String name : List.of(CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME, CanonicalSolversFactory.ASTAR_OD_MUA_MAX_NAME,
                    CanonicalSolversFactory.ASTAR_OD_MUA_IH_NAME, CanonicalSolversFactory.ASTAR_OD_MUA_MH_NAME)) {
                Solution sol = CanonicalSolversFactory.createSolver(name).solve(c.instance, new RunParametersBuilder()
                        .setTimeout(60_000).setInstanceReport(Metrics.newInstanceReport()).createRP());
                if (optimum == INF) {
                    assertNull(sol, name + " on infeasible " + c.name);
                } else {
                    assertNotNull(sol, name + " on " + c.name);
                    assertTrue(sol.solves(c.instance), name + " on " + c.name);
                    assertEquals(optimum, sol.MUA(), name + " on " + c.name);
                }
            }
            if (optimum == INF) infeasible++; else solvable++;
        }
        System.out.println("[paper-compliance] cross-solver: solvable=" + solvable + " infeasible=" + infeasible);
        assertTrue(solvable >= 20 && infeasible >= 1);
    }

    /**
     * The paper searches the plain joint-agent state space. A*+OD must return the same optimum and the same root
     * heuristic as a reference joint A* that expands complete joint moves only (no OD intermediate nodes).
     */
    @Test
    void odSolverMatchesReferenceJointAStarWithoutOd() {
        int compared = 0, infeasible = 0, excludedTimeouts = 0;
        long refFull = 0, odFull = 0, odAll = 0;
        StringBuilder table = new StringBuilder("[paper-compliance] case heuristic: refMUA odMUA | rootH | refFullExpanded odFullExpanded odAllExpanded\n");
        for (Case c : cases(40, 13)) {
            // The optimum does not depend on the guiding heuristic: one reference (ZERO) search per instance.
            ReferenceJointAStar.Result ref = ReferenceJointAStar.solve(c.instance, MuaHeuristicType.ZERO, 3_000_000);
            assertTrue(ref.completed(), "reference search hit its node cap on " + c.name);
            MuaProblemContext ctx = new MuaProblemContext(c.instance);
            for (MuaHeuristicType type : MuaHeuristicType.values()) {
                Environment.Metrics.InstanceReport report = Metrics.newInstanceReport();
                Solution sol = new AStarODMuaBuilder().setHeuristicType(type).createSolver().solve(c.instance,
                        new RunParametersBuilder().setTimeout(60_000).setInstanceReport(report).createRP());
                String where = c.name + " " + type;
                if (Integer.valueOf(1).equals(report.getIntegerValue(Environment.Metrics.InstanceReport.StandardFields.timedOut))) {
                    excludedTimeouts++;
                    table.append("  ").append(where).append(": TIMEOUT (excluded, not a pass)\n");
                    continue;
                }
                if (ref.mua() < 0) {
                    assertNull(sol, "OD found a solution where joint A* proves infeasibility: " + where);
                } else {
                    assertNotNull(sol, where);
                    assertTrue(sol.solves(c.instance), where);
                    assertEquals(ref.mua(), sol.MUA(), "optimal MUA: " + where);
                }
                long rootH = h(ctx, type, rootState(ctx));
                assertEquals(rootH, (long) report.getIntegerValue("MAPFUA_MA_root_h"), "root h: " + where);
                table.append("  ").append(where).append(": ").append(ref.mua()).append(' ').append(sol == null ? "-" : sol.MUA())
                        .append(" | ").append(rootH).append(" | ").append(ref.fullExpanded()).append(' ')
                        .append(report.getIntegerValue("MAPFUA_MA_full_joint_nodes")).append(' ')
                        .append(report.getIntegerValue(Environment.Metrics.InstanceReport.StandardFields.expandedNodes)).append('\n');
                if (ref.mua() < 0) infeasible++;
                refFull += ref.fullExpanded();
                odFull += report.getIntegerValue("MAPFUA_MA_full_joint_nodes");
                odAll += report.getIntegerValue(Environment.Metrics.InstanceReport.StandardFields.expandedNodes);
                compared++;
            }
        }
        System.out.println(table);
        System.out.println("[paper-compliance] reference-joint-A*: compared=" + compared + " infeasible=" + infeasible
                + " excludedTimeouts=" + excludedTimeouts + " expansions refJointFull=" + refFull
                + " odFull=" + odFull + " odAll=" + odAll);
        assertTrue(compared >= 160, "need at least as many completed comparisons as the scratch evidence (159)");
        assertTrue(excludedTimeouts <= 4, "too many timeouts to call this a correctness check: " + excludedTimeouts);
    }

    /** Test-only reference: A* over complete joint states (positions, movedUAs, visitedTargets), pure MUA cost. */
    static final class ReferenceJointAStar {
        record Result(long mua, long rootH, long fullExpanded, boolean completed) { }

        static Result solve(MAPF_Instance instance, MuaHeuristicType type, long nodeCap) {
            MuaProblemContext ctx = new MuaProblemContext(instance);
            MapfuaCaptureHeuristic heuristic = new MapfuaCaptureHeuristic(ctx, MapfuaObjective.MUA, type, false);
            record Node(MuaSearchState s, long g, long f, long id) { }
            PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingLong(Node::f).thenComparingLong(Node::id));
            Map<List<Object>, Long> best = new HashMap<>();
            I_Location[] start = new I_Location[ctx.agents.size()];
            BitSet visited0 = new BitSet();
            for (int i = 0; i < start.length; i++) {
                Agent a = ctx.agents.get(i);
                start[i] = ctx.map.getMapLocation(a.source);
                if (a.isUnassigned || start[i].getCoordinate().equals(a.target)) visited0.set(i);
            }
            MuaSearchState root = MuaSearchState.full(0, start, new BitSet(), visited0, 0, 0, null, null, 0);
            long rootH = heuristic.estimate(root);
            long serial = 0, expanded = 0;
            open.add(new Node(root, 0, rootH, serial++));
            best.put(List.of(Arrays.asList(start), new BitSet(), visited0), 0L);
            while (!open.isEmpty()) {
                Node n = open.poll();
                MuaSearchState s = n.s;
                if (!Objects.equals(best.get(List.of(Arrays.asList(s.positions), s.movedUAs, s.visitedTargets)), n.g)) continue;
                boolean goal = true;
                for (int a : ctx.assignedIndexes) goal &= s.visitedTargets.get(a);
                if (goal) return new Result(n.g, rootH, expanded, true);
                if (++expanded > nodeCap) return new Result(-1, rootH, expanded, false);
                for (I_Location[] next : jointMoves(ctx, s.positions, null, 0)) {
                    BitSet moved = (BitSet) s.movedUAs.clone();
                    BitSet visited = (BitSet) s.visitedTargets.clone();
                    int cost = 0;
                    for (int i = 0; i < next.length; i++) {
                        int ua = ctx.uaIndexByAgentIndex[i];
                        if (ua >= 0 && !next[i].equals(s.positions[i]) && !moved.get(ua)) {
                            moved.set(ua);
                            cost++;
                        }
                        if (next[i].getCoordinate().equals(ctx.agents.get(i).target)) visited.set(i);
                    }
                    List<Object> key = List.of(Arrays.asList(next), moved, visited);
                    Long old = best.get(key);
                    long g = n.g + cost;
                    if (old == null || g < old) {
                        best.put(key, g);
                        MuaSearchState child = MuaSearchState.full(s.time + 1, next, moved, visited, g, 0, null, null, 0);
                        open.add(new Node(child, g, g + heuristic.estimate(child), serial++));
                    }
                }
            }
            return new Result(-1, rootH, expanded, true);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Instances
    // ---------------------------------------------------------------------------------------------

    private record Case(String name, MAPF_Instance instance, long seed) { }

    /** Hand-made fixtures covering the required categories, plus random small grids. */
    private List<Case> cases(int randomCount, long seed) {
        List<Case> out = new ArrayList<>();
        out.add(new Case("no_ua_move", arbitrary("mapfua_ma_no_ua_move.scene", 2), 1));
        out.add(new Case("side_pocket(1 UA helps 2 AAs, AA interaction)", arbitrary("cbs_star_side_pocket.scene", 3), 2));
        out.add(new Case("idle_ua", arbitrary("audit_mua_side_pocket_idle_ua.scene", 4), 3));
        out.add(new Case("heur_pair(two distinct UAs, IH pair)", arbitrary("mapfua_heur_pair.scene", 4), 4));
        out.add(new Case("heur_max(cap 1 vs 2)", arbitrary("mapfua_heur_max.scene", 5), 5));
        out.add(new Case("heur_decoy(decoys)", arbitrary("mapfua_heur_decoy.scene", 6), 6));
        out.add(new Case("return_ua", arbitrary("mapfua_ma_return_ua.scene", 2), 7));
        out.add(new Case("two_vertex_swap(infeasible)", arbitrary("two_vertex_swap.scene", 2), 8));
        out.add(new Case("independent_regions", independentRegions(), 9));
        Random rnd = new Random(seed);
        int[][] shapes = {{3, 3}, {2, 4}, {3, 4}, {2, 3}};
        for (int i = 0; i < randomCount; i++) {
            int[] shape = shapes[i % shapes.length];
            int cells = shape[0] * shape[1];
            int total = Math.min(cells <= 9 ? 5 : 4, 2 + rnd.nextInt(3));
            int numAa = 1 + rnd.nextInt(Math.min(3, total));
            out.add(new Case("rand" + i + "_" + shape[0] + "x" + shape[1] + "_aa" + numAa + "_n" + total,
                    randomGrid(shape[0], shape[1], numAa, total - numAa, rnd), 100 + i));
        }
        return out;
    }

    static MAPF_Instance randomGrid(int w, int h, int numAa, int numUa, Random rnd) {
        Enum_MapLocationType[][] grid = new Enum_MapLocationType[w][h];
        for (Enum_MapLocationType[] col : grid) Arrays.fill(col, Enum_MapLocationType.EMPTY);
        List<Coordinate_2D> cells = new ArrayList<>();
        for (int x = 0; x < w; x++) for (int y = 0; y < h; y++) cells.add(new Coordinate_2D(x, y));
        List<Coordinate_2D> starts = new ArrayList<>(cells);
        java.util.Collections.shuffle(starts, rnd);
        Agent[] agents = new Agent[numAa + numUa];
        List<Coordinate_2D> goals;
        do {
            goals = new ArrayList<>(cells);
            java.util.Collections.shuffle(goals, rnd);
            goals = goals.subList(0, numAa);
        } while (!validGoals(starts, goals));
        for (int i = 0; i < numAa; i++) agents[i] = new Agent(i, starts.get(i), goals.get(i));
        for (int i = numAa; i < numAa + numUa; i++) agents[i] = new Agent(i, starts.get(i), starts.get(i));
        return new MAPF_Instance("rand", MapFactory.newSimple4Connected2D_GraphMap(grid), agents);
    }

    private static boolean validGoals(List<Coordinate_2D> starts, List<Coordinate_2D> goals) {
        for (int i = 0; i < goals.size(); i++) if (goals.get(i).equals(starts.get(i))) return false;
        return true;
    }

    /** Two disconnected-in-effect regions: two 1x3 corridors joined by nothing, each with its own AA and UA. */
    private MAPF_Instance independentRegions() {
        Enum_MapLocationType[][] grid = new Enum_MapLocationType[3][3];
        for (Enum_MapLocationType[] col : grid) Arrays.fill(col, Enum_MapLocationType.EMPTY);
        grid[0][1] = grid[1][1] = grid[2][1] = Enum_MapLocationType.WALL; // rows y=0 and y=2 are separate corridors
        Agent[] agents = {
                new Agent(0, new Coordinate_2D(0, 0), new Coordinate_2D(2, 0)),
                new Agent(1, new Coordinate_2D(0, 2), new Coordinate_2D(1, 2)),
                new Agent(2, new Coordinate_2D(1, 0), new Coordinate_2D(1, 0)),
                new Agent(3, new Coordinate_2D(2, 2), new Coordinate_2D(2, 2)),
        };
        return new MAPF_Instance("regions", MapFactory.newSimple4Connected2D_GraphMap(grid, false), agents);
    }

    // ---------------------------------------------------------------------------------------------
    // State sampling (full joint states and OD intermediate states, built as the solver builds them)
    // ---------------------------------------------------------------------------------------------

    private record Sample(MuaSearchState state, boolean partial, MuaSearchState base, I_Location[] prefixNext, int prefix) {
        @Override public String toString() {
            return (partial ? "OD[prefix=" + prefix + "]" : "FULL") + " pos=" + Arrays.toString(state.positions)
                    + " moved=" + state.movedUAs + " visited=" + state.visitedTargets;
        }
    }

    private List<Sample> sampleStates(MuaProblemContext ctx, Random rnd, int walks, int steps, boolean withPartial) {
        List<Sample> out = new ArrayList<>();
        MuaSearchState root = rootState(ctx);
        out.add(new Sample(root, false, null, null, 0));
        for (int w = 0; w < walks; w++) {
            MuaSearchState cur = root;
            for (int s = 0; s < steps; s++) {
                List<I_Location[]> moves = jointMoves(ctx, cur.positions, null, 0);
                if (moves.isEmpty()) break;
                I_Location[] next = moves.get(rnd.nextInt(moves.size()));
                int n = next.length;
                if (withPartial && n > 1) {
                    int prefix = 1 + rnd.nextInt(n - 1);
                    out.add(partialSample(ctx, cur, next, prefix));
                }
                cur = applyPrefix(ctx, cur, next, n);
                out.add(new Sample(cur, false, null, null, 0));
            }
        }
        return out;
    }

    /** Mirrors AStarODMuaSolver.expandAgent/partialHeuristic: prefix agents at next, the rest at current. */
    private Sample partialSample(MuaProblemContext ctx, MuaSearchState base, I_Location[] next, int prefix) {
        MuaSearchState synthetic = applyPrefix(ctx, base, next, prefix);
        return new Sample(synthetic, true, base, next, prefix);
    }

    private MuaSearchState applyPrefix(MuaProblemContext ctx, MuaSearchState base, I_Location[] next, int prefix) {
        I_Location[] pos = Arrays.copyOf(base.positions, base.positions.length);
        BitSet moved = (BitSet) base.movedUAs.clone();
        BitSet visited = (BitSet) base.visitedTargets.clone();
        for (int i = 0; i < prefix; i++) {
            pos[i] = next[i];
            int ua = ctx.uaIndexByAgentIndex[i];
            if (ua >= 0 && !next[i].equals(base.positions[i])) moved.set(ua);
            if (next[i].getCoordinate().equals(ctx.agents.get(i).target)) visited.set(i);
        }
        return MuaSearchState.full(base.time + 1, pos, moved, visited, 0, 0, null, null, 0);
    }

    /** Joint moves with vertex + swap legality (as AStarODMuaSolver.conflictsWithPrefix); agents < fixed keep fixedNext. */
    private static List<I_Location[]> jointMoves(MuaProblemContext ctx, I_Location[] cur, I_Location[] fixedNext, int fixed) {
        List<I_Location[]> out = new ArrayList<>();
        I_Location[] chosen = new I_Location[cur.length];
        for (int i = 0; i < fixed; i++) chosen[i] = fixedNext[i];
        enumerate(cur, fixed, chosen, out);
        return out;
    }

    private static void enumerate(I_Location[] cur, int idx, I_Location[] chosen, List<I_Location[]> out) {
        if (idx == cur.length) {
            out.add(Arrays.copyOf(chosen, chosen.length));
            return;
        }
        List<I_Location> moves = new ArrayList<>(cur[idx].outgoingEdges());
        moves.sort(Comparator.comparing(l -> l.getCoordinate().toString()));
        if (cur[idx].getType() != Enum_MapLocationType.NO_STOP) moves.add(cur[idx]);
        for (I_Location nxt : moves) {
            boolean bad = false;
            for (int j = 0; j < idx && !bad; j++) {
                bad = nxt.equals(chosen[j]) || (nxt.equals(cur[j]) && cur[idx].equals(chosen[j]));
            }
            if (bad) continue;
            chosen[idx] = nxt;
            enumerate(cur, idx + 1, chosen, out);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Exact oracle (brute-force Dijkstra over full joint states, pure MUA cost)
    // ---------------------------------------------------------------------------------------------

    private final class Oracle {
        final MuaProblemContext ctx;
        final Map<List<Object>, Long> memo = new HashMap<>();

        Oracle(MuaProblemContext ctx) {
            this.ctx = ctx;
        }

        long exact(MuaSearchState s) {
            List<Object> key = List.of(Arrays.asList(s.positions), s.movedUAs.clone(), s.visitedTargets.clone());
            Long cached = memo.get(key);
            if (cached != null) return cached;
            long v = dijkstra(s.positions, s.movedUAs, s.visitedTargets);
            memo.put(key, v);
            return v;
        }

        /** min over completions of the OD step: new UA moves of the suffix agents + exact cost afterwards. */
        long completionCost(Sample s) {
            long best = INF;
            for (I_Location[] next : jointMoves(ctx, s.base.positions, s.prefixNext, s.prefix)) {
                MuaSearchState after = applyPrefix(ctx, s.base, next, next.length);
                int suffixNewUa = after.movedUAs.cardinality() - s.state.movedUAs.cardinality();
                long rest = exact(after);
                if (rest != INF) best = Math.min(best, suffixNewUa + rest);
            }
            return best;
        }

        private long dijkstra(I_Location[] start, BitSet moved0, BitSet visited0) {
            record Node(I_Location[] pos, BitSet moved, BitSet visited, long g, long id) { }
            PriorityQueue<Node> open = new PriorityQueue<>(Comparator.comparingLong(Node::g).thenComparingLong(Node::id));
            Map<List<Object>, Long> best = new HashMap<>();
            long serial = 0;
            open.add(new Node(start, moved0, visited0, 0, serial++));
            best.put(List.of(Arrays.asList(start), moved0, visited0), 0L);
            while (!open.isEmpty()) {
                Node n = open.poll();
                if (best.get(List.of(Arrays.asList(n.pos), n.moved, n.visited)) != n.g) continue;
                boolean goal = true;
                for (int a : ctx.assignedIndexes) goal &= n.visited.get(a);
                if (goal) return n.g;
                for (I_Location[] next : jointMoves(ctx, n.pos, null, 0)) {
                    BitSet moved = (BitSet) n.moved.clone();
                    BitSet visited = (BitSet) n.visited.clone();
                    int cost = 0;
                    for (int i = 0; i < next.length; i++) {
                        int ua = ctx.uaIndexByAgentIndex[i];
                        if (ua >= 0 && !next[i].equals(n.pos[i]) && !moved.get(ua)) {
                            moved.set(ua);
                            cost++;
                        }
                        if (next[i].getCoordinate().equals(ctx.agents.get(i).target)) visited.set(i);
                    }
                    List<Object> key = List.of(Arrays.asList(next), moved, visited);
                    Long old = best.get(key);
                    if (old == null || n.g + cost < old) {
                        best.put(key, n.g + cost);
                        open.add(new Node(next, moved, visited, n.g + cost, serial++));
                    }
                }
            }
            return INF;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Reference implementations: verbatim semantics of the original (pre-683bc58) h1 / h4 / h3 code.
    // ---------------------------------------------------------------------------------------------

    private List<Integer> active(MuaProblemContext ctx, MuaSearchState s) {
        List<Integer> a = new ArrayList<>();
        for (int i : ctx.assignedIndexes) if (!s.visitedTargets.get(i)) a.add(i);
        return a;
    }

    private int uaAt(MuaProblemContext ctx, I_Location loc, BitSet staticUAs) {
        for (int i = staticUAs.nextSetBit(0); i >= 0; i = staticUAs.nextSetBit(i + 1)) {
            if (ctx.uaStartLocations[i].equals(loc)) return i;
        }
        return -1;
    }

    /** cap(ai): Dijkstra over (location, captured set) minimizing |captured|; entering an unmoved UA captures it. */
    private long refCap(MuaProblemContext ctx, I_Location start, I_Location target, BitSet moved) {
        BitSet staticUAs = ctx.staticUAs(moved);
        record N(I_Location loc, BitSet caps, long id) { }
        PriorityQueue<N> open = new PriorityQueue<>(Comparator.comparingInt((N n) -> n.caps.cardinality()).thenComparingLong(N::id));
        Set<List<Object>> closed = new HashSet<>();
        long serial = 0;
        open.add(new N(start, new BitSet(), serial++));
        while (!open.isEmpty()) {
            N n = open.poll();
            if (!closed.add(List.of(n.loc, n.caps))) continue;
            if (n.loc.equals(target)) return n.caps.cardinality();
            for (I_Location nxt : n.loc.outgoingEdges()) {
                BitSet caps = (BitSet) n.caps.clone();
                int ua = uaAt(ctx, nxt, staticUAs);
                if (ua >= 0) caps.set(ua);
                open.add(new N(nxt, caps, serial++));
            }
        }
        return INF;
    }

    private long refMax(MuaProblemContext ctx, MuaSearchState s) {
        long best = 0;
        for (int a : active(ctx, s)) {
            long cap = refCap(ctx, s.positions[a], ctx.targetOf(a), s.movedUAs);
            if (cap == INF) return ctx.uaStartLocations.length + 1L;
            best = Math.max(best, cap);
        }
        return best;
    }

    private boolean reachableAvoiding(MuaProblemContext ctx, I_Location start, I_Location target, BitSet obstacles) {
        if (uaAt(ctx, start, obstacles) >= 0 || uaAt(ctx, target, obstacles) >= 0) return false;
        ArrayDeque<I_Location> q = new ArrayDeque<>(List.of(start));
        Set<I_Location> seen = new HashSet<>(q);
        while (!q.isEmpty()) {
            I_Location c = q.poll();
            if (c.equals(target)) return true;
            for (I_Location n : c.outgoingEdges()) if (uaAt(ctx, n, obstacles) < 0 && seen.add(n)) q.add(n);
        }
        return false;
    }

    /** IH: smallest k such that some k-subset of unmoved UAs, once removed, lets every active AA reach its target. */
    private long refIH(MuaProblemContext ctx, MuaSearchState s) {
        BitSet staticUAs = ctx.staticUAs(s.movedUAs);
        int[] items = staticUAs.stream().toArray();
        List<Integer> act = active(ctx, s);
        for (int k = 0; k <= items.length; k++) {
            List<BitSet> subsets = new ArrayList<>();
            combinations(items, k, 0, new BitSet(), subsets);
            for (BitSet removed : subsets) {
                BitSet obstacles = (BitSet) staticUAs.clone();
                obstacles.andNot(removed);
                boolean all = true;
                for (int a : act) all &= reachableAvoiding(ctx, s.positions[a], ctx.targetOf(a), obstacles);
                if (all) return k;
            }
        }
        return ctx.uaStartLocations.length + 1L;
    }

    private void combinations(int[] items, int k, int from, BitSet cur, List<BitSet> out) {
        if (cur.cardinality() == k) {
            out.add((BitSet) cur.clone());
            return;
        }
        for (int i = from; i < items.length; i++) {
            cur.set(items[i]);
            combinations(items, k, i + 1, cur, out);
            cur.clear(items[i]);
        }
    }

    /** MH: plain uniform-cost joint search (FIFO tie-break) over (positions, captured, served), as the original h3. */
    private long refMH(MuaProblemContext ctx, MuaSearchState s) {
        List<Integer> act = active(ctx, s);
        if (act.isEmpty()) return 0;
        BitSet staticUAs = ctx.staticUAs(s.movedUAs);
        int m = act.size();
        I_Location[] start = new I_Location[m];
        BitSet served0 = new BitSet();
        for (int i = 0; i < m; i++) {
            start[i] = s.positions[act.get(i)];
            if (start[i].getCoordinate().equals(ctx.agents.get(act.get(i)).target)) served0.set(i);
        }
        record N(I_Location[] pos, BitSet caps, BitSet served, long g, long id) { }
        PriorityQueue<N> open = new PriorityQueue<>(Comparator.comparingLong(N::g).thenComparingLong(N::id));
        Map<List<Object>, Long> best = new HashMap<>();
        long serial = 0;
        open.add(new N(start, new BitSet(), served0, 0, serial++));
        best.put(List.of(Arrays.asList(start), new BitSet(), served0), 0L);
        while (!open.isEmpty()) {
            N n = open.poll();
            if (!Objects.equals(best.get(List.of(Arrays.asList(n.pos), n.caps, n.served)), n.g)) continue;
            if (n.served.cardinality() == m) return n.g;
            List<I_Location[]> moves = new ArrayList<>();
            enumerateWithWait(n.pos, 0, new I_Location[m], moves);
            for (I_Location[] next : moves) {
                BitSet caps = (BitSet) n.caps.clone();
                int newCaps = 0;
                BitSet served = (BitSet) n.served.clone();
                for (int i = 0; i < m; i++) {
                    int ua = uaAt(ctx, next[i], staticUAs);
                    if (ua >= 0 && !caps.get(ua)) {
                        caps.set(ua);
                        newCaps++;
                    }
                    if (next[i].getCoordinate().equals(ctx.agents.get(act.get(i)).target)) served.set(i);
                }
                List<Object> key = List.of(Arrays.asList(next), caps, served);
                Long old = best.get(key);
                if (old == null || n.g + newCaps < old) {
                    best.put(key, n.g + newCaps);
                    open.add(new N(next, caps, served, n.g + newCaps, serial++));
                }
            }
        }
        return ctx.uaStartLocations.length + m + 1L;
    }

    private void enumerateWithWait(I_Location[] cur, int idx, I_Location[] chosen, List<I_Location[]> out) {
        if (idx == cur.length) {
            out.add(Arrays.copyOf(chosen, chosen.length));
            return;
        }
        List<I_Location> moves = new ArrayList<>(cur[idx].outgoingEdges());
        moves.add(cur[idx]);
        for (I_Location nxt : moves) {
            boolean bad = false;
            for (int j = 0; j < idx && !bad; j++) bad = nxt.equals(chosen[j]) || (cur[idx].equals(chosen[j]) && nxt.equals(cur[j]));
            if (bad) continue;
            chosen[idx] = nxt;
            enumerateWithWait(cur, idx + 1, chosen, out);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------------------------

    private long h(MuaProblemContext ctx, MuaHeuristicType type, MuaSearchState s) {
        return new MapfuaCaptureHeuristic(ctx, MapfuaObjective.MUA, type, false).estimate(s);
    }

    private I_Location[] starts(MuaProblemContext ctx) {
        I_Location[] s = new I_Location[ctx.agents.size()];
        for (int i = 0; i < s.length; i++) s[i] = ctx.map.getMapLocation(ctx.agents.get(i).source);
        return s;
    }

    private MuaSearchState rootState(MuaProblemContext ctx) {
        BitSet visited = new BitSet();
        I_Location[] pos = starts(ctx);
        for (int i = 0; i < pos.length; i++) {
            Agent a = ctx.agents.get(i);
            if (a.isUnassigned || pos[i].getCoordinate().equals(a.target)) visited.set(i);
        }
        return MuaSearchState.full(0, pos, new BitSet(), visited, 0, 0, null, null, 0);
    }

    private static final Map<String, MAPF_Instance> CACHE = new HashMap<>();

    private MAPF_Instance arbitrary(String scenario, int agents) {
        synchronized (CACHE) {
            MAPF_Instance cached = CACHE.get(scenario + agents);
            if (cached != null) return cached;
            String dir = IO_Manager.buildPath(new String[]{IO_Manager.testResources_Directory, "Instances", "ArbitraryGraphs"});
            String prefix = scenario.replace(".scene", "");
            InstanceBuilder_ArbitraryGraph builder = new InstanceBuilder_ArbitraryGraph();
            builder.prepareInstances(prefix, new InstanceManager.Moving_AI_Path(
                            IO_Manager.buildPath(new String[]{dir, prefix + ".edgelist"}),
                            IO_Manager.buildPath(new String[]{dir, scenario})),
                    new InstanceProperties(null, -1, new int[]{agents}));
            MAPF_Instance instance = builder.getNextExistingInstance();
            assertNotNull(instance, scenario);
            CACHE.put(scenario + agents, instance);
            return instance;
        }
    }
}
