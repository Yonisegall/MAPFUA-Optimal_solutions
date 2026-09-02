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

import java.util.List;
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
        MAPF_Instance instance = twoVertexSwapInstance();
        I_Solver solver = CanonicalSolversFactory.createCBSStarMUASolver();
        Solution solution = solver.solve(instance, new RunParametersBuilder()
                .setTimeout(10_000)
                .setInstanceReport(Metrics.newInstanceReport())
                .createRP());
        assertNull(solution);
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
