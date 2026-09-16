package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.InstanceBuilders.InstanceBuilder_ArbitraryGraph;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Solvers.CanonicalSolversFactory;
import BasicMAPF.Solvers.I_Solver;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.Metrics;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * PERFORMANCE STRESS TEST for CBS*, not a correctness or merge-scope test.
 *
 * "cbs_star_double_side_pocket" is two structurally disjoint copies of "cbs_star_side_pocket" (vertices
 * {@code L,C,R,P,Q} and {@code L2,C2,R2,P2,Q2} share no edge or vertex). Each copy independently requires exactly
 * one UA move, so MUA* = 2.
 *
 * This class must NOT assert anything about how many meta-agents CBS* forms, how large they get, or whether it
 * merges agents across the two independent components. Algorithm 2 of the CBS* paper this solver implements
 * defines {@code S = { m in M | loc(m,t1) = loc(m,t2) } } over the WHOLE meta-agent set M with no connectivity
 * restriction, and the paper's own completeness proof relies on exactly the degenerate case where a fully-repeated
 * global joint configuration makes S = M. A run that merges agents from both of this instance's independent
 * components (as confirmed against Algorithm 2) is expected, paper-faithful behavior — not a defect. This instance
 * is kept purely as a stress case for that mechanism (two independent "trouble spots" can each trigger merges whose
 * scope isn't limited to the interacting component), reporting runtime and the CBS*-specific counters as a
 * diagnostic. The only assertion is a correctness one unrelated to merge scope: IF CBS* returns a solution at all,
 * its MUA must still be exactly correct.
 */
class CBSStarDoubleSidePocketStressTest {

    @Test
    void doubleSidePocketStressProbeReportsCountersAndPreservesMUACorrectnessIfSolved() {
        MAPF_Instance instance = loadDoubleSidePocket();
        I_Solver solver = CanonicalSolversFactory.createSolver(CanonicalSolversFactory.CBS_STAR_MUA_NAME);
        InstanceReport report = Metrics.newInstanceReport();

        long start = System.currentTimeMillis();
        Solution solution = solver.solve(instance, new RunParametersBuilder().setTimeout(10_000).setInstanceReport(report).createRP());
        long elapsedMs = System.currentTimeMillis() - start;

        System.out.println("[double-side-pocket STRESS TEST, not a correctness check] solved=" + (solution != null)
                + " elapsedMs=" + elapsedMs
                + " maxMetaAgentSize=" + report.getIntegerValue(CBSStarMUA_Solver.METRIC_MAX_META_AGENT_SIZE)
                + " metaAgentCreations=" + report.getIntegerValue(CBSStarMUA_Solver.METRIC_META_AGENT_CREATIONS)
                + " duplicateDetections=" + report.getIntegerValue(CBSStarMUA_Solver.METRIC_DUPLICATE_DETECTIONS)
                + " duplicatePruned=" + report.getIntegerValue(CBSStarMUA_Solver.METRIC_DUPLICATE_PRUNED)
                + " conflictsTotal=" + report.getIntegerValue(CBSStarMUA_Solver.METRIC_CONFLICTS_TOTAL)
                + " peakOpenSize=" + report.getIntegerValue(CBSStarMUA_Solver.METRIC_PEAK_OPEN_SIZE));

        if (solution != null) {
            assertEquals(2, solution.MUA(), "true MUA* for two independent one-UA-move pockets is 2");
        }
    }

    private MAPF_Instance loadDoubleSidePocket() {
        String dir = IO_Manager.buildPath(new String[]{IO_Manager.testResources_Directory, "Instances", "ArbitraryGraphs"});
        String prefix = "cbs_star_double_side_pocket";
        InstanceManager.Moving_AI_Path path = new InstanceManager.Moving_AI_Path(
                IO_Manager.buildPath(new String[]{dir, prefix + ".edgelist"}),
                IO_Manager.buildPath(new String[]{dir, prefix + ".scene"}));
        for (int i = 0; i < 3; i++) {
            InstanceBuilder_ArbitraryGraph builder = new InstanceBuilder_ArbitraryGraph();
            builder.prepareInstances(prefix, path, new InstanceProperties(null, -1, new int[]{6}));
            MAPF_Instance instance = builder.getNextExistingInstance();
            if (instance != null) return instance;
        }
        throw new IllegalStateException("could not load " + prefix);
    }
}
