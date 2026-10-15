package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.DataTypesAndStructures.Timeout;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.InstanceBuilders.InstanceBuilder_ArbitraryGraph;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Solvers.CanonicalSolversFactory;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import BasicMAPF.Solvers.I_Solver;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.Metrics;
import org.junit.jupiter.api.Test;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.*;

class CBSStarMUA_SolverTest {
    private static MAPF_Instance cachedSidePocket;
    private static MAPF_Instance cachedTwoVertexSwap;

    @Test
    void metaAgentPartitionMergesAreCompleteAndImmutableForSiblings() {
        MAPF_Instance instance = sidePocketInstance();
        MetaAgentPartition root = MetaAgentPartition.singletonPartition(instance.agents);
        assertEquals(3, root.metaAgents().size());
        for (MetaAgent group : root.metaAgents()) {
            assertEquals(1, group.size());
        }

        MetaAgent first = root.containing(instance.agents.get(0));
        MetaAgent second = root.containing(instance.agents.get(1));
        MetaAgentPartition sibling = root;
        MetaAgentPartition merged = root.merge(List.of(first, second));

        assertEquals(3, sibling.metaAgents().size());
        assertEquals(2, merged.containing(instance.agents.get(0)).size());
        assertSameAgents(instance.agents, merged);
    }

    @Test
    void jointLowLevelAllowsUaAwayAndCountsUaOnce() {
        MAPF_Instance instance = sidePocketInstance();
        Agent ua = instance.agents.get(2);
        CBSStarJointLowLevel lowLevel = new CBSStarJointLowLevel(instance,
                TransientMAPF.TransientMAPFSettings.defaultTransientMAPF,
                new Timeout(Timeout.getCurrentTimeMS_NSAccuracy(), 10_000));
        ConstraintSet constraints = new ConstraintSet();
        constraints.add(new Constraint(ua, 1, instance.map.getMapLocation(ua.source)));
        Solution solution = lowLevel.solve(new MetaAgent(List.of(ua)), constraints);

        assertNotNull(solution);
        SingleAgentPlan plan = solution.getPlanFor(ua);
        assertEquals(1, plan.mua());
        assertNotEquals(instance.map.getMapLocation(ua.source), plan.getLastMove().currLocation);
    }

    @Test
    void waitingUaContributesZero() {
        MAPF_Instance instance = sidePocketInstance();
        Agent ua = instance.agents.get(2);
        CBSStarJointLowLevel lowLevel = new CBSStarJointLowLevel(instance,
                TransientMAPF.TransientMAPFSettings.defaultTransientMAPF,
                new Timeout(Timeout.getCurrentTimeMS_NSAccuracy(), 10_000));
        Solution solution = lowLevel.solve(new MetaAgent(List.of(ua)), new ConstraintSet());
        assertNotNull(solution);
        assertEquals(0, solution.MUA());
    }

    @Test
    void twoVertexSwapTerminatesWithNoSolutionAfterMerge() {
        // Regression for Timed-Out-vs-genuinely-infeasible reporting: this instance is providably unsolvable
        // (handleLoopMerge exhausts every merge option), so it must finish almost instantly, well within the
        // 10s budget - Solved=0 here must NOT also be reported as Timed Out=1.
        MAPF_Instance instance = twoVertexSwapInstance();
        I_Solver solver = CanonicalSolversFactory.createCBSStarMUASolver();
        InstanceReport report = Metrics.newInstanceReport();
        Solution solution = solver.solve(instance, new RunParametersBuilder()
                .setTimeout(10_000)
                .setInstanceReport(report)
                .createRP());
        assertNull(solution);
        assertEquals(0, report.getIntegerValue(InstanceReport.StandardFields.solved));
        assertEquals(0, report.getIntegerValue(InstanceReport.StandardFields.timedOut),
                "a genuinely unsolvable instance solved well inside its timeout must not be reported as Timed Out");
    }

    @Test
    void reportsTimedOutWhenSharedDeadlineExpiresInsideNestedLowLevelSearch() {
        // Regression for the CBS*/timeout-reporting gap: runAlgorithm's two "return null" exit points (root
        // generation failing, and the main loop's open list emptying out because every remaining branch's
        // low-level replan/merge returned null) do not themselves call checkTimeout() - so if the actual reason
        // a nested CBSStarJointLowLevel.solve() call returned null was that the *shared* Timeout had expired
        // (not that the meta-agent was genuinely unsolvable), abortedForTimeout previously stayed false and the
        // run was misreported as a clean Solved=0 rather than Timed Out=1. cbs_star_double_side_pocket is the
        // known pathological CBS* merge-storm instance (see CBSStarDoubleSidePocketStressTest / project memory)
        // that reliably burns through a short deadline via nested low-level calls well before its own open list
        // would naturally empty out.
        MAPF_Instance instance = loadArbitrary("cbs_star_double_side_pocket", 6);
        assertNotNull(instance);
        I_Solver solver = CanonicalSolversFactory.createCBSStarMUASolver();
        InstanceReport report = Metrics.newInstanceReport();
        Solution solution = solver.solve(instance, new RunParametersBuilder()
                .setTimeout(300)
                .setInstanceReport(report)
                .createRP());
        assertNull(solution, "300ms is not enough for CBS* to solve the double-side-pocket merge storm");
        assertEquals(0, report.getIntegerValue(InstanceReport.StandardFields.solved));
        assertEquals(1, report.getIntegerValue(InstanceReport.StandardFields.timedOut),
                "hitting the shared deadline inside a nested low-level call must still surface as Timed Out=1");
    }

    @Test
    void sidePocketInstanceParsesAndSolvesWithMUAOne() {
        MAPF_Instance instance = sidePocketInstance();
        assertEquals(3, instance.agents.size());

        I_Solver solver = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.CBS_STAR_MUA_NAME);
        InstanceReport report = Metrics.newInstanceReport();
        Solution solution = solver.solve(instance, new RunParametersBuilder()
                .setTimeout(10_000)
                .setInstanceReport(report)
                .createRP());

        assertNotNull(solution);
        assertCBSStarMAPFUASolution(instance, solution);
        assertEquals(1, solution.MUA());
        assertEquals(1, independentlyCountMovedUAs(solution));
        assertEquals(CanonicalSolversFactory.CBS_STAR_MUA_NAME, solver.getName());
        assertEquals(1, report.getIntegerValue("MUA"));
    }

    @Test
    void simpleNoUaMovementInstanceReturnsMUAZero() {
        MAPF_Instance instance = sidePocketInstance();
        Agent aa = instance.agents.get(0);
        MAPF_Instance single = instance.getSubproblemFor(Set.of(aa));
        I_Solver solver = CanonicalSolversFactory.createCBSStarMUASolver();
        Solution solution = solver.solve(single, new RunParametersBuilder()
                .setTimeout(10_000)
                .setInstanceReport(Metrics.newInstanceReport())
                .createRP());
        assertNotNull(solution);
        assertEquals(0, solution.MUA());
    }

    @Test
    void cbsStarPureObjectivesMatchAStarODZeroBaselines() {
        MAPF_Instance instance = sidePocketInstance();
        Object[][] cases = {
                {CanonicalSolversFactory.CBS_STAR_MUA_NAME, CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME},
                {CanonicalSolversFactory.CBS_STAR_FUEL_NAME, CanonicalSolversFactory.ASTAR_OD_FUEL_ZERO_NAME},
                {CanonicalSolversFactory.CBS_STAR_SST_NAME, CanonicalSolversFactory.ASTAR_OD_SST_ZERO_NAME}
        };
        for (Object[] c : cases) {
            String cbsName = (String) c[0];
            String astarName = (String) c[1];
            Solution cbs = CanonicalSolversFactory.createSolver(cbsName).solve(instance, new RunParametersBuilder()
                    .setTimeout(10_000)
                    .setInstanceReport(Metrics.newInstanceReport())
                    .createRP());
            Solution astar = CanonicalSolversFactory.createSolver(astarName).solve(instance, new RunParametersBuilder()
                    .setTimeout(10_000)
                    .setInstanceReport(Metrics.newInstanceReport())
                    .createRP());
            assertNotNull(cbs, cbsName);
            assertNotNull(astar, astarName);
            assertCBSStarMAPFUASolution(instance, cbs);
            assertEquals(objectiveValue(cbsName, astar), objectiveValue(cbsName, cbs), cbsName);
        }
    }

    @Test
    void cbsStarAndAStarODZeroAgreeOnMUAAcrossSmallInstancesWheneverBothSolve() {
        Map<String, Integer> instancesToAgentCounts = new LinkedHashMap<>();
        instancesToAgentCounts.put("cbs_star_side_pocket", 3);
        instancesToAgentCounts.put("mapfua_ma_no_ua_move", 2);
        instancesToAgentCounts.put("mapfua_ma_return_ua", 2);
        instancesToAgentCounts.put("two_vertex_swap", 2);
        // included even though it is a known stress case for CBS*'s paper-faithful merge mechanism (see
        // CBSStarDoubleSidePocketStressTest): "whenever both solve" naturally skips it if CBS* times out.
        instancesToAgentCounts.put("cbs_star_double_side_pocket", 6);

        for (Map.Entry<String, Integer> entry : instancesToAgentCounts.entrySet()) {
            String instanceName = entry.getKey();
            MAPF_Instance instance = loadArbitrary(instanceName, entry.getValue());

            Solution cbsSolution = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.CBS_STAR_MUA_NAME)
                    .solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());
            Solution aStarSolution = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME)
                    .solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());

            if (cbsSolution != null && aStarSolution != null) {
                assertEquals(aStarSolution.MUA(), cbsSolution.MUA(),
                        instanceName + ": CBSStarMUA and AStarODMUAZero must agree on optimal MUA when both solve");
            } else {
                System.out.println("[cross-check] skipping " + instanceName + " (cbsSolved="
                        + (cbsSolution != null) + ", aStarZeroSolved=" + (aStarSolution != null) + ")");
            }
        }
    }

    @Test
    void transientAgentMayLeaveTargetSoAnotherAgentCanVisitIt() {
        // Regression for the CBS*/MAPFUA transient-goal false-UNSAT bug: X -- Y -- Z, a0 targets Y from the left,
        // a1 targets Y from the right. No UAs are involved. The only joint solutions require one agent to visit Y
        // and then vacate it so the other agent can visit Y afterward - e.g. a0: X->Y->X, a1: Z->Z->Y. Trimming a0's
        // plan down to "X->Y" (its first target visit) and then re-checking under classical stay-forever conflict
        // semantics falsely turns this valid solution into an apparent conflict at Y.
        MAPF_Instance instance = sharedTargetLineInstance();
        Agent a0 = instance.agents.get(0);
        Agent a1 = instance.agents.get(1);

        Solution cbsSolution = CanonicalSolversFactory.createCBSStarMUASolver().solve(instance,
                new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());
        Solution aStarSolution = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME)
                .solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());

        // 1. CBSStarMUA returns non-null: this instance IS solvable under transient MAPFUA goal semantics.
        assertNotNull(cbsSolution, "CBSStarMUA must solve X-Y-Z with both AAs targeting the shared middle vertex; "
                + "it is solvable under transient (visit-once, may-leave-afterward) goal semantics");

        // 2. Returned solution is valid under transient MAPF semantics (conflict-free; each AA's target only needs
        // to be visited at some point, not to be the plan's final position).
        assertTrue(cbsSolution.solves(instance), "returned solution must be conflict-free and satisfy every "
                + "agent's source/target under transient semantics");

        // 3. Both assigned agents visit Y.
        assertTrue(visitsTarget(cbsSolution.getPlanFor(a0)), "a0 must visit its target Y");
        assertTrue(visitsTarget(cbsSolution.getPlanFor(a1)), "a1 must visit its target Y");

        // 4. MUA == 0: there are no UAs in this instance, and AAs never contribute to MUA no matter how much they move.
        assertEquals(0, cbsSolution.MUA());

        // 5. At least one agent must have a real move after first visiting its target - that escape move is exactly
        // what keeps the joint solution conflict-free (both agents cannot occupy Y at the same time forever).
        assertTrue(hasPostTargetMove(cbsSolution.getPlanFor(a0)) || hasPostTargetMove(cbsSolution.getPlanFor(a1)),
                "at least one AA must move again after first visiting Y, or the two AAs would collide at Y forever");

        // 6. CBSStarMUA and AStarODMUAZero agree.
        assertNotNull(aStarSolution, "AStarODMUAZero must also solve this instance");
        assertEquals(0, aStarSolution.MUA());
        assertEquals(aStarSolution.MUA(), cbsSolution.MUA());
    }

    @Test
    void transientAgentVacatesHubSoASecondAgentCanPassThrough() {
        // Second, non-symmetric transient-goal regression: a branching star (hub B with three spokes A, C, D; D is
        // otherwise unused) instead of a plain line. a0 goes A->B and a1 goes C->B: both target the same hub node,
        // so whichever agent arrives first must eventually vacate B so the other agent can also visit it (neither
        // can just "wait its turn" indefinitely, exactly as in the X-Y-Z case) - but here on a degree-3 branching
        // topology instead of a 3-node path, so this exercises the same semantic fix without being a symmetric
        // clone of the first regression.
        MAPF_Instance instance = sharedTargetHubInstance();
        Agent a0 = instance.agents.get(0);
        Agent a1 = instance.agents.get(1);

        Solution cbsSolution = CanonicalSolversFactory.createCBSStarMUASolver().solve(instance,
                new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());
        Solution aStarSolution = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME)
                .solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(Metrics.newInstanceReport()).createRP());

        assertNotNull(cbsSolution, "CBSStarMUA must solve this instance: a0 and a1 share target hub B, so it is "
                + "solvable under transient goal semantics (visit-once, may-leave-afterward)");
        assertTrue(cbsSolution.solves(instance));
        assertTrue(visitsTarget(cbsSolution.getPlanFor(a0)), "a0 must visit its target B");
        assertTrue(visitsTarget(cbsSolution.getPlanFor(a1)), "a1 must visit its target B");
        assertEquals(0, cbsSolution.MUA());
        assertTrue(hasPostTargetMove(cbsSolution.getPlanFor(a0)) || hasPostTargetMove(cbsSolution.getPlanFor(a1)),
                "at least one AA must move again after first visiting B, or the two AAs would collide at B forever");

        assertNotNull(aStarSolution, "AStarODMUAZero must also solve this instance");
        assertEquals(0, aStarSolution.MUA());
        assertEquals(aStarSolution.MUA(), cbsSolution.MUA());
    }

    private MAPF_Instance sharedTargetLineInstance() {
        MAPF_Instance instance = loadArbitrary("cbs_star_shared_target_line", 2);
        assertNotNull(instance);
        return instance;
    }

    private MAPF_Instance sharedTargetHubInstance() {
        MAPF_Instance instance = loadArbitrary("cbs_star_shared_target_hub", 2);
        assertNotNull(instance);
        return instance;
    }

    private boolean visitsTarget(SingleAgentPlan plan) {
        for (Move move : plan) {
            if (move.prevLocation.getCoordinate().equals(plan.agent.target)
                    || move.currLocation.getCoordinate().equals(plan.agent.target)) {
                return true;
            }
        }
        return false;
    }

    /**
     * @return true if the plan contains a real move (not a wait) strictly after the first time the agent visits
     * its target.
     */
    private boolean hasPostTargetMove(SingleAgentPlan plan) {
        boolean visited = false;
        for (Move move : plan) {
            if (visited && !move.prevLocation.equals(move.currLocation)) {
                return true;
            }
            if (move.prevLocation.getCoordinate().equals(plan.agent.target)
                    || move.currLocation.getCoordinate().equals(plan.agent.target)) {
                visited = true;
            }
        }
        return false;
    }

    private MAPF_Instance sidePocketInstance() {
        if (cachedSidePocket != null) return cachedSidePocket;
        MAPF_Instance instance = loadArbitrary("cbs_star_side_pocket", 3);
        assertNotNull(instance);
        cachedSidePocket = instance;
        return cachedSidePocket;
    }

    private MAPF_Instance twoVertexSwapInstance() {
        if (cachedTwoVertexSwap != null) return cachedTwoVertexSwap;
        MAPF_Instance instance = loadArbitrary("two_vertex_swap", 2);
        assertNotNull(instance);
        cachedTwoVertexSwap = instance;
        return cachedTwoVertexSwap;
    }

    private MAPF_Instance loadArbitrary(String prefix, int agents) {
        String dir = IO_Manager.buildPath(new String[]{IO_Manager.testResources_Directory, "Instances", "ArbitraryGraphs"});
        InstanceManager.Moving_AI_Path path = new InstanceManager.Moving_AI_Path(
                IO_Manager.buildPath(new String[]{dir, prefix + ".edgelist"}),
                IO_Manager.buildPath(new String[]{dir, prefix + ".scene"}));
        for (int i = 0; i < 3; i++) {
            InstanceBuilder_ArbitraryGraph builder = new InstanceBuilder_ArbitraryGraph();
            builder.prepareInstances(prefix, path, new InstanceProperties(null, -1, new int[]{agents}));
            MAPF_Instance instance = builder.getNextExistingInstance();
            if (instance != null) return instance;
        }
        return null;
    }

    private void assertSameAgents(List<Agent> expected, MetaAgentPartition partition) {
        for (Agent agent : expected) {
            assertTrue(partition.containing(agent).contains(agent));
        }
    }

    private void assertCBSStarMAPFUASolution(MAPF_Instance instance, Solution solution) {
        assertNull(solution.firstConflict(false, false));
        for (Agent agent : instance.agents) {
            SingleAgentPlan plan = solution.getPlanFor(agent);
            assertNotNull(plan);
            assertEquals(instance.map.getMapLocation(agent.source), plan.getFirstMove().prevLocation);
            if (!agent.isUnassigned) {
                boolean visited = false;
                for (Move move : plan) {
                    visited |= move.prevLocation.getCoordinate().equals(agent.target)
                            || move.currLocation.getCoordinate().equals(agent.target);
                }
                assertTrue(visited, "Assigned agent did not visit its target: " + agent);
            }
        }
    }

    private int independentlyCountMovedUAs(Solution solution) {
        int count = 0;
        for (SingleAgentPlan plan : solution) {
            if (!plan.agent.isUnassigned) continue;
            boolean moved = false;
            for (Move move : plan) {
                moved |= !move.prevLocation.equals(move.currLocation);
            }
            if (moved) count++;
        }
        return count;
    }

    private int objectiveValue(String solverName, Solution solution) {
        if (solverName.contains("MUA")) return solution.MUA();
        if (solverName.contains("Fuel")) return solution.FUEL();
        return solution.SST();
    }
}
