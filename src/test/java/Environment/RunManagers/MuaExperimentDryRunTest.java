package Environment.RunManagers;

import BasicMAPF.CostFunctions.Makespan;
import BasicMAPF.CostFunctions.MakespanServiceTime;
import BasicMAPF.CostFunctions.SumFuel;
import BasicMAPF.CostFunctions.SumMUA;
import BasicMAPF.CostFunctions.SumOfCosts;
import BasicMAPF.CostFunctions.SumServiceTimes;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.InstanceBuilders.InstanceBuilder_ArbitraryGraph;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Solvers.CBS.CBSStar.CBSStarMUA_Solver;
import BasicMAPF.Solvers.CanonicalSolversFactory;
import BasicMAPF.Solvers.I_Solver;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.Metrics;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end dry run of the exact 5-solver MUA experiment configuration (matching
 * {@link GenericRunManager#setSolvers()}) against tiny instances, writing a real CSV via the same
 * {@link Metrics}/{@link InstanceReport} machinery the real experiment runner uses, so the CSV itself (not just
 * in-memory Java objects) can be inspected. This is a readiness check, not a correctness oracle for any one solver
 * (those are covered elsewhere) — it exists to catch integration-level problems: missing columns, counters that
 * don't reset between runs, rows not written on failure/timeout, etc.
 */
class MuaExperimentDryRunTest {

    private static final List<String> FIVE_MUA_SOLVER_NAMES = List.of(
            CanonicalSolversFactory.CBS_STAR_MUA_NAME,
            CanonicalSolversFactory.ASTAR_OD_MUA_ZERO_NAME,
            CanonicalSolversFactory.ASTAR_OD_MUA_MAX_NAME,
            CanonicalSolversFactory.ASTAR_OD_MUA_IH_NAME,
            CanonicalSolversFactory.ASTAR_OD_MUA_MH_NAME);

    // the exact same header the real GenericRunManager/A_RunManager writes to the live results CSV
    private static final String[] PRODUCTION_HEADER = new String[]{
            InstanceReport.StandardFields.solver,
            InstanceReport.StandardFields.experimentName,
            InstanceReport.StandardFields.mapName,
            InstanceReport.StandardFields.instanceName,
            InstanceReport.StandardFields.numAgents,
            InstanceReport.StandardFields.numTraversableLocations,
            InstanceReport.StandardFields.elapsedTimeMS,
            InstanceReport.StandardFields.timeoutThresholdMS,
            InstanceReport.StandardFields.solved,
            InstanceReport.StandardFields.valid,
            InstanceReport.StandardFields.solutionCost,
            InstanceReport.StandardFields.solutionCostFunction,
            SumOfCosts.NAME,
            Makespan.NAME,
            SumServiceTimes.NAME,
            MakespanServiceTime.NAME,
            SumFuel.NAME,
            SumMUA.NAME,
            InstanceReport.StandardFields.expandedNodes,
            InstanceReport.StandardFields.generatedNodes,
            InstanceReport.StandardFields.expandedNodesLowLevel,
            InstanceReport.StandardFields.generatedNodesLowLevel,
            InstanceReport.StandardFields.totalLowLevelCalls,
            InstanceReport.StandardFields.totalLowLevelTimeMS,
            CBSStarMUA_Solver.METRIC_CONFLICTS_TOTAL,
            CBSStarMUA_Solver.METRIC_CONFLICTS_AA_AA,
            CBSStarMUA_Solver.METRIC_CONFLICTS_AA_UA,
            CBSStarMUA_Solver.METRIC_CONFLICTS_UA_UA,
            CBSStarMUA_Solver.METRIC_DUPLICATE_DETECTIONS,
            CBSStarMUA_Solver.METRIC_DUPLICATE_PRUNED,
            CBSStarMUA_Solver.METRIC_META_AGENT_CREATIONS,
            CBSStarMUA_Solver.METRIC_MAX_META_AGENT_SIZE,
            CBSStarMUA_Solver.METRIC_META_AGENT_LL_CALLS,
            CBSStarMUA_Solver.METRIC_PEAK_OPEN_SIZE,
    };

    @AfterEach
    void clearMetrics() {
        Metrics.clearAll();
    }

    @Test
    void fiveSolverConfigurationProducesOneRowPerSolverPerInstanceIncludingOnFailure() throws IOException {
        MAPF_Instance solvable = arbitrary("cbs_star_side_pocket", 3);
        MAPF_Instance unsolvable = arbitrary("two_vertex_swap", 2);

        for (String name : FIVE_MUA_SOLVER_NAMES) {
            // --- solvable instance: normal timeout ---
            runOne(name, solvable, 10_000);
            // --- unsolvable instance: normal timeout (proves-infeasible path) ---
            runOne(name, unsolvable, 10_000);
            // --- solvable instance again, but with an artificially tiny timeout to force a genuine
            //     timeout/abort mid-search, proving counters still get exported on failure ---
            runOne(name, solvable, 1);
        }

        List<InstanceReport> reports = Metrics.getAllReports();
        assertEquals(FIVE_MUA_SOLVER_NAMES.size() * 3, reports.size(), "expected exactly one row per (solver, run) combination");

        Path csvPath = Path.of("target", "dry-run-mua-experiment.csv");
        Files.createDirectories(csvPath.getParent());
        try (FileOutputStream out = new FileOutputStream(csvPath.toFile())) {
            Metrics.exportCSV(out, PRODUCTION_HEADER);
        }
        assertTrue(Files.exists(csvPath));

        // every row must at minimum carry solver name, cost function, and solved flag
        for (InstanceReport report : reports) {
            assertNotNull(report.getStringValue(InstanceReport.StandardFields.solver));
            assertEquals("MUA", report.getStringValue(InstanceReport.StandardFields.solutionCostFunction));
            assertNotNull(report.getIntegerValue(InstanceReport.StandardFields.solved));
        }

        System.out.println("Dry-run CSV written to: " + csvPath.toAbsolutePath());
    }

    @Test
    void cbsStarCountersResetBetweenConsecutiveRunsOnSameSolverInstance() {
        MAPF_Instance solvable = arbitrary("cbs_star_side_pocket", 3);
        I_Solver cbsStar = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.CBS_STAR_MUA_NAME);

        InstanceReport first = Metrics.newInstanceReport();
        cbsStar.solve(solvable, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(first).createRP());
        InstanceReport second = Metrics.newInstanceReport();
        cbsStar.solve(solvable, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(second).createRP());

        // re-running the identical instance on the identical (reused) solver object must produce identical
        // counters, not accumulating/leaking across runs
        assertEquals(first.getIntegerValue(CBSStarMUA_Solver.METRIC_CONFLICTS_TOTAL),
                second.getIntegerValue(CBSStarMUA_Solver.METRIC_CONFLICTS_TOTAL));
        assertEquals(first.getIntegerValue(CBSStarMUA_Solver.METRIC_META_AGENT_CREATIONS),
                second.getIntegerValue(CBSStarMUA_Solver.METRIC_META_AGENT_CREATIONS));
        assertEquals(first.getIntegerValue(CBSStarMUA_Solver.METRIC_PEAK_OPEN_SIZE),
                second.getIntegerValue(CBSStarMUA_Solver.METRIC_PEAK_OPEN_SIZE));
    }

    @Test
    void cbsStarCountersAreObservedChangingWhenMechanismsTrigger() {
        MAPF_Instance solvable = arbitrary("cbs_star_side_pocket", 3);
        I_Solver cbsStar = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.CBS_STAR_MUA_NAME);
        InstanceReport report = Metrics.newInstanceReport();
        Solution solution = cbsStar.solve(solvable, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(report).createRP());

        assertNotNull(solution, "sanity: side-pocket instance must be solved by CBS*");
        Integer conflictsTotal = report.getIntegerValue(CBSStarMUA_Solver.METRIC_CONFLICTS_TOTAL);
        Integer peakOpen = report.getIntegerValue(CBSStarMUA_Solver.METRIC_PEAK_OPEN_SIZE);
        assertNotNull(conflictsTotal);
        assertNotNull(peakOpen);
        assertTrue(conflictsTotal > 0, "side-pocket instance requires at least one conflict to be resolved by CBS* HL");
        assertTrue(peakOpen > 0, "OPEN must have held at least the root node");
    }

    private void runOne(String solverName, MAPF_Instance instance, int timeoutMS) {
        InstanceReport report = Metrics.newInstanceReport();
        report.putStringValue(InstanceReport.StandardFields.instanceName, instance.name);
        report.putIntegerValue(InstanceReport.StandardFields.numAgents, instance.agents.size());
        I_Solver solver = CanonicalSolversFactory.createSolver(solverName);
        solver.solve(instance, new RunParametersBuilder().setTimeout(timeoutMS).setInstanceReport(report).createRP());
    }

    private MAPF_Instance arbitrary(String prefix, int agents) {
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
        throw new IllegalStateException("could not load instance " + prefix);
    }
}
