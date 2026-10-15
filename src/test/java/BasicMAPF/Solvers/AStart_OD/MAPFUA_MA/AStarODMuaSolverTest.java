package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.InstanceBuilders.InstanceBuilder_ArbitraryGraph;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.CanonicalSolversFactory;
import BasicMAPF.Solvers.I_Solver;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.Metrics;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class AStarODMuaSolverTest {
    private static final Map<String, MAPF_Instance> CACHED_INSTANCES = new HashMap<>();

    @Test
    void objectiveStepCostsArePurePrimaryCosts() {
        assertEquals(1, MapfuaObjective.MUA.stepCost(1, 99, 99));
        assertEquals(0, MapfuaObjective.MUA.stepCost(0, 99, 99));
        assertEquals(7, MapfuaObjective.FUEL.stepCost(1, 7, 99));
        assertEquals(3, MapfuaObjective.SST.stepCost(1, 7, 3));
    }

    @Test
    void stateKeyDistinguishesMovedVisitedAndPartialPrefix() {
        MAPF_Instance instance = sidePocket();
        MuaProblemContext context = new MuaProblemContext(instance);
        I_Location[] positions = starts(context);
        BitSet moved = new BitSet();
        BitSet visited = new BitSet();
        visited.set(2);
        MuaSearchState full = MuaSearchState.full(0, positions, moved, visited, 0, 0, null, null, 0);
        BitSet moved2 = new BitSet();
        moved2.set(0);
        MuaSearchState movedDifferent = MuaSearchState.full(0, positions, moved2, visited, 0, 0, null, null, 1);
        BitSet visited2 = (BitSet) visited.clone();
        visited2.set(0);
        MuaSearchState visitedDifferent = MuaSearchState.full(0, positions, moved, visited2, 0, 0, null, null, 2);

        assertNotEquals(full.key(), movedDifferent.key());
        assertNotEquals(full.key(), visitedDifferent.key());

        I_Location[] next = new I_Location[3];
        Move[] moves = new Move[3];
        next[0] = instance.map.getMapLocation(instance.agents.get(0).source);
        moves[0] = new Move(instance.agents.get(0), 1, next[0], next[0]);
        MuaSearchState partialOne = MuaSearchState.partial(full, 1, next, moved, visited, moves, 0, 3, 0, 0);
        MuaSearchState partialTwo = MuaSearchState.partial(full, 2, next, moved, visited, moves, 0, 4, 0, 0);
        assertNotEquals(partialOne.key(), partialTwo.key());
    }

    @Test
    void allAStarODObjectivesSolveSidePocketWithSamePrimaryCost() {
        MAPF_Instance instance = sidePocket();
        for (String name : allObjectiveHeuristicSolverNames()) {
            Solution solution = CanonicalSolversFactory.createSolver(name).solve(instance, new RunParametersBuilder()
                    .setTimeout(10_000)
                    .setInstanceReport(Metrics.newInstanceReport())
                    .createRP());
            assertNotNull(solution, name);
            assertTrue(solution.solves(instance), name);
            assertNoConflicts(instance, solution);
            if (name.contains("MUA")) assertEquals(1, solution.MUA(), name);
            if (name.contains("Fuel")) assertEquals(7, solution.FUEL(), name);
            if (name.contains("SST")) assertEquals(solution.SST(), reportedCost(name, solution), name);
            assertEquals(1, countMovedOriginalUAs(solution), name);
            assertNotEquals(instance.map.getMapLocation(instance.agents.get(2).source),
                    solution.getPlanFor(instance.agents.get(2)).getLastMove().currLocation);
        }
    }

    @Test
    void noRequiredUaMovementReturnsMuaZero() {
        MAPF_Instance instance = arbitrary("mapfua_ma_no_ua_move.scene", 2);
        Solution solution = CanonicalSolversFactory.createAStarODMUAMaxSolver().solve(instance, new RunParametersBuilder()
                .setTimeout(10_000)
                .setInstanceReport(Metrics.newInstanceReport())
                .createRP());
        assertNotNull(solution);
        assertEquals(0, solution.MUA());
    }

    @Test
    void hValuesNeverExceedExactSmallSidePocketCost() {
        MAPF_Instance instance = sidePocket();
        MuaProblemContext context = new MuaProblemContext(instance);
        I_Location[] positions = starts(context);
        BitSet moved = new BitSet();
        BitSet visited = new BitSet();
        visited.set(2);
        MuaSearchState state = MuaSearchState.full(0, positions, moved, visited, 0, 0, null, null, 0);
        long exact = exactRemainingCost(context, state, MapfuaObjective.MUA);
        for (MuaHeuristicType type : MuaHeuristicType.values()) {
            long h = new MapfuaCaptureHeuristic(context, MapfuaObjective.MUA, type, false).estimate(state);
            assertTrue(h <= exact, type + " returned " + h);
        }
    }

    @Test
    void informedHeuristicsAreAdmissibleAgainstIndependentSmallOracle() {
        MAPF_Instance instance = sidePocket();
        MuaProblemContext context = new MuaProblemContext(instance);
        List<MuaSearchState> states = new ArrayList<>();

        I_Location[] rootPositions = starts(context);
        BitSet rootVisited = new BitSet();
        rootVisited.set(2);
        states.add(MuaSearchState.full(0, rootPositions, new BitSet(), rootVisited, 0, 0, null, null, 0));

        I_Location[] afterUaMoves = starts(context);
        afterUaMoves[2] = instance.map.getMapLocation(instance.agents.get(2).target);
        BitSet movedUa = new BitSet();
        movedUa.set(0);
        states.add(MuaSearchState.full(1, afterUaMoves, movedUa, rootVisited, 0, 0, null, null, 1));

        I_Location[] aaVisitedAndLeft = starts(context);
        aaVisitedAndLeft[0] = instance.map.getMapLocation(instance.agents.get(0).source);
        BitSet servedOne = (BitSet) rootVisited.clone();
        servedOne.set(0);
        states.add(MuaSearchState.full(2, aaVisitedAndLeft, movedUa, servedOne, 0, 0, null, null, 2));

        for (MuaSearchState state : states) {
            for (MapfuaObjective objective : MapfuaObjective.values()) {
                long exact = exactRemainingCost(context, state, objective);
                for (MuaHeuristicType type : List.of(MuaHeuristicType.MAX, MuaHeuristicType.INCREASING,
                        MuaHeuristicType.MERGE)) {
                    long h = new MapfuaCaptureHeuristic(context, objective, type, false).estimate(state);
                    assertTrue(h <= exact, objective + " " + type + " returned " + h + " > " + exact);
                }
            }
        }
    }

    @Test
    void everyHeuristicReturnsZeroAtGoal() {
        MAPF_Instance instance = sidePocket();
        MuaProblemContext context = new MuaProblemContext(instance);
        I_Location[] positions = starts(context);
        positions[0] = instance.map.getMapLocation(instance.agents.get(0).target);
        positions[1] = instance.map.getMapLocation(instance.agents.get(1).target);
        BitSet visited = new BitSet();
        visited.set(0);
        visited.set(1);
        visited.set(2);
        MuaSearchState goal = MuaSearchState.full(4, positions, new BitSet(), visited, 0, 0, null, null, 0);
        for (MapfuaObjective objective : MapfuaObjective.values()) {
            for (MuaHeuristicType type : MuaHeuristicType.values()) {
                assertEquals(0, new MapfuaCaptureHeuristic(context, objective, type, false).estimate(goal), objective + " " + type);
            }
        }
    }

    @Test
    void mergeHonorsTransientVisitedTargetAfterAgentLeavesGoal() {
        MAPF_Instance instance = sidePocket();
        MuaProblemContext context = new MuaProblemContext(instance);
        I_Location[] positions = starts(context);
        positions[0] = instance.map.getMapLocation(instance.agents.get(0).source);
        positions[1] = instance.map.getMapLocation(instance.agents.get(1).target);
        BitSet visited = new BitSet();
        visited.set(0);
        visited.set(1);
        visited.set(2);
        MuaSearchState state = MuaSearchState.full(5, positions, new BitSet(), visited, 0, 0, null, null, 0);
        assertEquals(0, new MapfuaCaptureHeuristic(context, MapfuaObjective.MUA, MuaHeuristicType.MERGE, false).estimate(state));
    }

    @Test
    void uaMovesMultipleTimesAndReturnsButCountsOnce() {
        MAPF_Instance instance = sidePocket();
        Agent aa = instance.agents.get(0);
        Agent ua = instance.agents.get(2);
        I_Location l = instance.map.getMapLocation(aa.source);
        I_Location c = l.outgoingEdges().get(0);
        I_Location p = instance.map.getMapLocation(ua.source);
        I_Location q = p.outgoingEdges().stream().filter(loc -> !loc.equals(c)).findFirst().orElseThrow();

        SingleAgentPlan aaPlan = new SingleAgentPlan(aa);
        aaPlan.addMove(new Move(aa, 1, l, c));
        SingleAgentPlan uaPlan = new SingleAgentPlan(ua);
        uaPlan.addMove(new Move(ua, 1, p, q));
        uaPlan.addMove(new Move(ua, 2, q, p));

        Solution solution = new Solution();
        solution.putPlan(aaPlan);
        solution.putPlan(uaPlan);

        assertEquals(1, solution.MUA());
        assertEquals(3, solution.FUEL());
        assertEquals(p, uaPlan.getLastMove().currLocation);
    }

    @Test
    void rdcUsesStaticUaSignatureAndFallsBackToCoordinateDistance() {
        MAPF_Instance instance = sidePocket();
        MuaProblemContext context = new MuaProblemContext(instance);
        BitSet noMovedUAs = new BitSet();
        BitSet staticUAs = context.staticUAs(noMovedUAs);
        I_Location left = instance.map.getMapLocation(instance.agents.get(0).source);
        I_Location right = instance.map.getMapLocation(instance.agents.get(0).target);
        I_Location pocket = instance.map.getMapLocation(instance.agents.get(2).source);
        I_Location q = instance.map.getMapLocation(instance.agents.get(2).target);

        assertEquals(2, context.rdc.query(left, right, staticUAs));
        assertFalse(context.rdc.reachableWithoutFallback(pocket, q, staticUAs));
        assertEquals(0, context.rdc.query(pocket, q, staticUAs));

        BitSet uaMoved = new BitSet();
        uaMoved.set(0);
        assertTrue(context.rdc.reachableWithoutFallback(pocket, q, context.staticUAs(uaMoved)));
        assertTrue(context.rdc.builds >= 3);
        assertTrue(context.rdc.manhattanFallbacks >= 1);
    }

    @Test
    void lazyAndEagerReturnSameCostAndFactoryNamesWork() {
        MAPF_Instance instance = sidePocket();
        AStarODMuaSolver eager = new AStarODMuaBuilder().setHeuristicType(MuaHeuristicType.MAX).createSolver();
        AStarODMuaSolver lazy = new AStarODMuaBuilder().setHeuristicType(MuaHeuristicType.MAX).setLazyEvaluation(true).createSolver();
        Solution eagerSol = eager.solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());
        InstanceReport lazyReport = Metrics.newInstanceReport();
        Solution lazySol = lazy.solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(lazyReport).createRP());
        assertNotNull(eagerSol);
        assertNotNull(lazySol);
        assertEquals(eagerSol.MUA(), lazySol.MUA());
        assertEquals(eagerSol.FUEL(), lazySol.FUEL());
        assertTrue(lazyReport.getIntegerValue("MAPFUA_MA_lazy_inherited") > 0);

        for (String name : allObjectiveHeuristicSolverNames()) {
            I_Solver solver = CanonicalSolversFactory.createSolver(name);
            assertEquals(name, solver.getName());
        }
    }

    @Test
    void twoVertexSwapTerminatesWithNoSolution() {
        MAPF_Instance instance = arbitrary("two_vertex_swap.scene", 2);
        Solution solution = CanonicalSolversFactory.createAStarODMUAZeroSolver().solve(instance, new RunParametersBuilder()
                .setTimeout(10_000)
                .setInstanceReport(Metrics.newInstanceReport())
                .createRP());
        assertNull(solution);
    }

    // ---------------------------------------------------------------------------------------------
    // Source-paper heuristics: ZERO, MAX, IH (increasing), MH (merge)
    // ---------------------------------------------------------------------------------------------

    @Test
    void canonicalMuaSolverNamesAreExactlyZeroMaxIhMh() {
        assertEquals("AStarODMUAZero", CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME);
        assertEquals("AStarODMUAMax", CanonicalSolversFactory.ASTAR_OD_MUA_MAX_NAME);
        assertEquals("AStarODMUAIH", CanonicalSolversFactory.ASTAR_OD_MUA_IH_NAME);
        assertEquals("AStarODMUAMH", CanonicalSolversFactory.ASTAR_OD_MUA_MH_NAME);
        assertEquals(List.of(MuaHeuristicType.ZERO, MuaHeuristicType.MAX, MuaHeuristicType.INCREASING, MuaHeuristicType.MERGE),
                List.of(MuaHeuristicType.values()));
        for (String legacy : List.of("AStarODMUAH1Max", "AStarODMUAH2EnhancedMax", "AStarODMUAH3Joint", "AStarODMUAH4RandomPick")) {
            assertThrows(IllegalArgumentException.class, () -> CanonicalSolversFactory.createSolver(legacy), legacy);
        }
    }

    @Test
    void zeroHeuristicIsAlwaysZero() {
        for (MAPF_Instance instance : List.of(sidePocket(), heurMax(), heurPair(), heurDecoy())) {
            MuaProblemContext context = new MuaProblemContext(instance);
            assertEquals(0, h(context, MuaHeuristicType.ZERO, rootState(context)));
        }
    }

    @Test
    void maxTakesMaximumOfIndependentPerAaCaptureCounts() {
        // a0 must capture U1 (cap=1); a1 must capture V1 and V2 (cap=2).
        MuaProblemContext context = new MuaProblemContext(heurMax());
        MuaSearchState root = rootState(context);
        assertEquals(2, h(context, MuaHeuristicType.MAX, root));
        assertEquals(3, h(context, MuaHeuristicType.INCREASING, root));
        assertEquals(3, h(context, MuaHeuristicType.MERGE, root));
    }

    @Test
    void increasingNeedsPairWhenNoSingleUaRemovalHelpsAllAas() {
        // a0 is blocked only by X, a1 only by Y: no single removal frees both, the pair {X, Y} does.
        MuaProblemContext context = new MuaProblemContext(heurPair());
        MuaSearchState root = rootState(context);
        assertEquals(1, h(context, MuaHeuristicType.MAX, root));
        assertEquals(2, h(context, MuaHeuristicType.INCREASING, root));
        assertEquals(2, h(context, MuaHeuristicType.MERGE, root));
    }

    @Test
    void increasingFindsTheSingleRelevantUaAmongDecoys() {
        // Three decoy UAs (enumerated first) never block anybody; only the UA on X blocks both AAs.
        MuaProblemContext context = new MuaProblemContext(heurDecoy());
        MuaSearchState root = rootState(context);
        assertEquals(4, context.uaStartLocations.length);
        assertEquals(1, h(context, MuaHeuristicType.INCREASING, root));
        assertEquals(1, h(context, MuaHeuristicType.MAX, root));
        assertEquals(1, h(context, MuaHeuristicType.MERGE, root));
    }

    @Test
    void mergeReflectsAaInteractionThatIndependentReachabilityIgnores() {
        // Side pocket: each AA alone reaches its target capture-free, but the two AAs must swap
        // along L-C-R, which jointly requires entering the pocket P and capturing its UA.
        MuaProblemContext context = new MuaProblemContext(sidePocket());
        MuaSearchState root = rootState(context);
        assertEquals(0, h(context, MuaHeuristicType.MAX, root));
        assertEquals(0, h(context, MuaHeuristicType.INCREASING, root));
        assertEquals(1, h(context, MuaHeuristicType.MERGE, root));
        assertEquals(1, exactRemainingCost(context, root, MapfuaObjective.MUA));
    }

    @Test
    void alreadyMovedUaIsNeverChargedAgain() {
        // heurMax UA indexes: 0=U1, 1=V1, 2=V2. A moved UA sitting back on its start is still paid for.
        MuaProblemContext context = new MuaProblemContext(heurMax());
        BitSet movedV1 = new BitSet();
        movedV1.set(1);
        MuaSearchState s1 = state(starts(context), movedV1, initialVisited(context));
        assertEquals(1, h(context, MuaHeuristicType.MAX, s1));
        assertEquals(2, h(context, MuaHeuristicType.INCREASING, s1));
        assertEquals(2, h(context, MuaHeuristicType.MERGE, s1));
        assertEquals(2, exactRemainingCost(context, s1, MapfuaObjective.MUA));

        BitSet movedU1V1 = (BitSet) movedV1.clone();
        movedU1V1.set(0);
        MuaSearchState s2 = state(starts(context), movedU1V1, initialVisited(context));
        assertEquals(1, h(context, MuaHeuristicType.MAX, s2));
        assertEquals(1, h(context, MuaHeuristicType.INCREASING, s2));
        assertEquals(1, h(context, MuaHeuristicType.MERGE, s2));

        BitSet allMoved = new BitSet();
        allMoved.set(0, 3);
        MuaSearchState s3 = state(starts(context), allMoved, initialVisited(context));
        for (MuaHeuristicType type : MuaHeuristicType.values()) assertEquals(0, h(context, type, s3), type.name());
    }

    @Test
    void alreadyServedAaCreatesNoRemainingRequirement() {
        // heurPair: a1 has already visited T2 and walked back to S2; only a0's need for X remains.
        MuaProblemContext context = new MuaProblemContext(heurPair());
        BitSet visited = initialVisited(context);
        visited.set(1);
        MuaSearchState s = state(starts(context), new BitSet(), visited);
        assertEquals(1, h(context, MuaHeuristicType.MAX, s));
        assertEquals(1, h(context, MuaHeuristicType.INCREASING, s));
        assertEquals(1, h(context, MuaHeuristicType.MERGE, s));
        assertEquals(1, exactRemainingCost(context, s, MapfuaObjective.MUA));

        // heurMax: a1 (the cap=2 agent) already served -> only a0's single capture remains.
        MuaProblemContext maxContext = new MuaProblemContext(heurMax());
        BitSet maxVisited = initialVisited(maxContext);
        maxVisited.set(1);
        MuaSearchState m = state(starts(maxContext), new BitSet(), maxVisited);
        assertEquals(1, h(maxContext, MuaHeuristicType.MAX, m));
        assertEquals(1, h(maxContext, MuaHeuristicType.INCREASING, m));
        assertEquals(1, h(maxContext, MuaHeuristicType.MERGE, m));
    }

    @Test
    void sourceHeuristicsAreAdmissibleAndOrderedMaxLeIhLeMh() {
        Random random = new Random(12345);
        for (MAPF_Instance instance : List.of(sidePocket(), heurMax(), heurPair(), heurDecoy())) {
            MuaProblemContext context = new MuaProblemContext(instance);
            List<MuaSearchState> states = new ArrayList<>();
            states.add(rootState(context));
            for (int walk = 0; walk < 3; walk++) {
                MuaSearchState current = rootState(context);
                for (int step = 0; step < 4; step++) {
                    current = randomSuccessor(context, current, random);
                    states.add(current);
                }
            }
            for (MuaSearchState s : states) {
                long exact = exactRemainingCost(context, s, MapfuaObjective.MUA);
                long max = h(context, MuaHeuristicType.MAX, s);
                long ih = h(context, MuaHeuristicType.INCREASING, s);
                long mh = h(context, MuaHeuristicType.MERGE, s);
                String where = instance.name + " moved=" + s.movedUAs + " visited=" + s.visitedTargets
                        + " MAX=" + max + " IH=" + ih + " MH=" + mh + " exact=" + exact;
                assertTrue(0 <= max && max <= exact, where);
                assertTrue(0 <= ih && ih <= exact, where);
                assertTrue(0 <= mh && mh <= exact, where);
                assertTrue(max <= ih && ih <= mh, where);
            }
        }
    }

    @Test
    void sourceHeuristicSolversReturnOptimalMuaOnFixtures() {
        List<Object[]> fixtures = List.of(
                new Object[]{"mapfua_heur_max.scene", 5, 3},
                new Object[]{"mapfua_heur_pair.scene", 4, 2},
                new Object[]{"mapfua_heur_decoy.scene", 6, 1});
        for (Object[] fixture : fixtures) {
            String scenario = (String) fixture[0];
            MAPF_Instance instance = arbitrary(scenario, (Integer) fixture[1]);
            for (String name : List.of(CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME, CanonicalSolversFactory.ASTAR_OD_MUA_MAX_NAME,
                    CanonicalSolversFactory.ASTAR_OD_MUA_IH_NAME, CanonicalSolversFactory.ASTAR_OD_MUA_MH_NAME)) {
                Solution solution = CanonicalSolversFactory.createSolver(name).solve(instance, new RunParametersBuilder()
                        .setTimeout(30_000)
                        .setInstanceReport(Metrics.newInstanceReport())
                        .createRP());
                assertNotNull(solution, name + " on " + scenario);
                assertTrue(solution.solves(instance), name + " on " + scenario);
                assertEquals((int) (Integer) fixture[2], solution.MUA(), name + " on " + scenario);
            }
        }
    }

    private long h(MuaProblemContext context, MuaHeuristicType type, MuaSearchState state) {
        return new MapfuaCaptureHeuristic(context, MapfuaObjective.MUA, type, false).estimate(state);
    }

    private BitSet initialVisited(MuaProblemContext context) {
        BitSet visited = new BitSet();
        for (int i = 0; i < context.agents.size(); i++) {
            Agent agent = context.agents.get(i);
            if (agent.isUnassigned || agent.source.equals(agent.target)) visited.set(i);
        }
        return visited;
    }

    private MuaSearchState rootState(MuaProblemContext context) {
        return state(starts(context), new BitSet(), initialVisited(context));
    }

    private MuaSearchState state(I_Location[] positions, BitSet moved, BitSet visited) {
        return MuaSearchState.full(0, positions, moved, visited, 0, 0, null, null, 0);
    }

    private MuaSearchState randomSuccessor(MuaProblemContext context, MuaSearchState s, Random random) {
        List<I_Location[]> jointMoves = new ArrayList<>();
        enumerateExactJointMoves(context, s.positions, 0, new I_Location[s.positions.length], jointMoves);
        I_Location[] next = jointMoves.get(random.nextInt(jointMoves.size()));
        BitSet moved = (BitSet) s.movedUAs.clone();
        BitSet visited = (BitSet) s.visitedTargets.clone();
        for (int i = 0; i < next.length; i++) {
            int ua = context.uaIndexByAgentIndex[i];
            if (ua >= 0 && !next[i].equals(s.positions[i])) moved.set(ua);
            if (next[i].getCoordinate().equals(context.agents.get(i).target)) visited.set(i);
        }
        return MuaSearchState.full(s.time + 1, next, moved, visited, 0, 0, null, null, 0);
    }

    private MAPF_Instance heurMax() {
        return arbitrary("mapfua_heur_max.scene", 5);
    }

    private MAPF_Instance heurPair() {
        return arbitrary("mapfua_heur_pair.scene", 4);
    }

    private MAPF_Instance heurDecoy() {
        return arbitrary("mapfua_heur_decoy.scene", 6);
    }

    private int reportedCost(String name, Solution solution) {
        if (name.contains("MUA")) return solution.MUA();
        if (name.contains("Fuel")) return solution.FUEL();
        return solution.SST();
    }

    private List<String> allObjectiveHeuristicSolverNames() {
        return List.of(
                CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME,
                CanonicalSolversFactory.ASTAR_OD_MUA_MAX_NAME,
                CanonicalSolversFactory.ASTAR_OD_MUA_IH_NAME,
                CanonicalSolversFactory.ASTAR_OD_MUA_MH_NAME,
                CanonicalSolversFactory.ASTAR_OD_FUEL_ZERO_NAME,
                CanonicalSolversFactory.ASTAR_OD_FUEL_MAX_NAME,
                CanonicalSolversFactory.ASTAR_OD_FUEL_IH_NAME,
                CanonicalSolversFactory.ASTAR_OD_FUEL_MH_NAME,
                CanonicalSolversFactory.ASTAR_OD_SST_ZERO_NAME,
                CanonicalSolversFactory.ASTAR_OD_SST_MAX_NAME,
                CanonicalSolversFactory.ASTAR_OD_SST_IH_NAME,
                CanonicalSolversFactory.ASTAR_OD_SST_MH_NAME);
    }

    private MAPF_Instance sidePocket() {
        return arbitrary("cbs_star_side_pocket.scene", 3);
    }

    private MAPF_Instance arbitrary(String scenario, int agents) {
        MAPF_Instance cached = CACHED_INSTANCES.get(scenario + agents);
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
        CACHED_INSTANCES.put(scenario + agents, instance);
        return instance;
    }

    private I_Location[] starts(MuaProblemContext context) {
        I_Location[] starts = new I_Location[context.agents.size()];
        for (int i = 0; i < starts.length; i++) {
            starts[i] = context.map.getMapLocation(context.agents.get(i).source);
        }
        return starts;
    }

    private int countMovedOriginalUAs(Solution solution) {
        int count = 0;
        for (SingleAgentPlan plan : solution) {
            if (plan.agent.isUnassigned && plan.mua() == 1) count++;
        }
        return count;
    }

    private void assertNoConflicts(MAPF_Instance instance, Solution solution) {
        assertNull(solution.firstConflict(), "solution has a conflict");
        for (Agent agent : instance.agents) {
            SingleAgentPlan plan = solution.getPlanFor(agent);
            assertNotNull(plan);
            if (!agent.isUnassigned) assertTrue(plan.containsTarget(), agent.toString());
        }
    }

    private long exactRemainingCost(MuaProblemContext context, MuaSearchState start, MapfuaObjective objective) {
        PriorityQueue<ExactNode> open = new PriorityQueue<>((a, b) -> {
            int byCost = Long.compare(a.g, b.g);
            if (byCost != 0) return byCost;
            return Long.compare(a.id, b.id);
        });
        Map<ExactKey, Long> best = new HashMap<>();
        long[] serial = {0};
        ExactNode root = new ExactNode(start.positions, start.movedUAs, start.visitedTargets, 0, serial[0]++);
        open.add(root);
        best.put(root.key(), root.g);
        while (!open.isEmpty()) {
            ExactNode current = open.poll();
            if (!Objects.equals(best.get(current.key()), current.g)) continue;
            if (allAssignedServed(context, current.visited)) return current.g;
            List<I_Location[]> jointMoves = new ArrayList<>();
            enumerateExactJointMoves(context, current.positions, 0, new I_Location[current.positions.length], jointMoves);
            for (I_Location[] next : jointMoves) {
                BitSet moved = (BitSet) current.moved.clone();
                BitSet visited = (BitSet) current.visited.clone();
                int newUa = 0;
                int fuel = 0;
                int sst = 0;
                for (int idx : context.assignedIndexes) {
                    if (!current.visited.get(idx)) sst++;
                }
                for (int i = 0; i < next.length; i++) {
                    boolean move = !next[i].equals(current.positions[i]);
                    if (move) fuel++;
                    int ua = context.uaIndexByAgentIndex[i];
                    if (ua >= 0 && move && !moved.get(ua)) {
                        moved.set(ua);
                        newUa++;
                    }
                    if (next[i].getCoordinate().equals(context.agents.get(i).target)) visited.set(i);
                }
                ExactNode child = new ExactNode(next, moved, visited, current.g + objective.stepCost(newUa, fuel, sst), serial[0]++);
                Long old = best.get(child.key());
                if (old == null || child.g < old) {
                    best.put(child.key(), child.g);
                    open.add(child);
                }
            }
        }
        fail("independent oracle found no solution");
        return 0;
    }

    private boolean allAssignedServed(MuaProblemContext context, BitSet visited) {
        for (int agentIndex : context.assignedIndexes) {
            if (!visited.get(agentIndex)) return false;
        }
        return true;
    }

    private void enumerateExactJointMoves(MuaProblemContext context, I_Location[] current, int index,
                                          I_Location[] chosen, List<I_Location[]> result) {
        if (index == current.length) {
            result.add(Arrays.copyOf(chosen, chosen.length));
            return;
        }
        List<I_Location> moves = new ArrayList<>(current[index].outgoingEdges());
        moves.sort(Comparator.comparing(l -> l.getCoordinate().toString()));
        moves.add(current[index]);
        for (I_Location next : moves) {
            boolean conflict = false;
            for (int j = 0; j < index; j++) {
                if (next.equals(chosen[j]) || (current[index].equals(chosen[j]) && next.equals(current[j]))) {
                    conflict = true;
                    break;
                }
            }
            if (conflict) continue;
            chosen[index] = next;
            enumerateExactJointMoves(context, current, index + 1, chosen, result);
        }
    }

    private record ExactKey(I_Location[] positions, BitSet moved, BitSet visited) {
        ExactKey {
            positions = Arrays.copyOf(positions, positions.length);
            moved = (BitSet) moved.clone();
            visited = (BitSet) visited.clone();
        }
        @Override public boolean equals(Object o) {
            return o instanceof ExactKey that
                    && Arrays.equals(positions, that.positions)
                    && moved.equals(that.moved)
                    && visited.equals(that.visited);
        }
        @Override public int hashCode() {
            int result = Arrays.hashCode(positions);
            result = 31 * result + moved.hashCode();
            result = 31 * result + visited.hashCode();
            return result;
        }
    }

    private record ExactNode(I_Location[] positions, BitSet moved, BitSet visited, long g, long id) {
        ExactNode {
            positions = Arrays.copyOf(positions, positions.length);
            moved = (BitSet) moved.clone();
            visited = (BitSet) visited.clone();
        }
        ExactKey key() {
            return new ExactKey(positions, moved, visited);
        }
    }
}
