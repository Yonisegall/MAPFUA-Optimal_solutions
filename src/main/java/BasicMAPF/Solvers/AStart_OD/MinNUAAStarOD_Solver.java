package BasicMAPF.Solvers.AStart_OD;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import BasicMAPF.CostFunctions.SumServiceTimes;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Solvers.A_Solver;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;

/**
 * Minimises NUA using A*+OD alone (no LaCAM oracle).
 *
 * Approach:
 *   for k = 0, 1, 2, ..., |UA|:
 *       for each subset of k UA agents ("movers"):
 *           freeze the rest as static
 *           run A*+OD
 *           if solution found → return (this is optimal NUA = k)
 *
 * Since A*+OD is complete and optimal, the first k that yields a solution
 * is guaranteed to be the minimum NUA.
 */
public class MinNUAAStarOD_Solver extends A_Solver {

    private int odMinNUA       = -1;
    private long odTotalTimeMS =  0;

    @Override
    protected void init(MAPF_Instance instance, RunParameters parameters) {
        super.init(instance, parameters);
        this.name = "AStarOD_Dynamic_UA";
        this.setConfiguredCostFunctionName("NUA");
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {

        List<Agent> uaAgents = getUAAgents(instance);

        // reset per-run metrics
        this.odMinNUA       = -1;
        this.odTotalTimeMS  =  0;

        System.out.println("=======================================================");
        System.out.println("[NUA_OD] Starting. Total UA agents: " + uaAgents.size());
        System.out.println("=======================================================");

        // k = number of UA that are ALLOWED to move
        for (int k = 0; k <= uaAgents.size(); k++) {
            if (checkTimeout()) {
                System.out.println("[NUA_OD] TIMEOUT before finishing k=" + k);
                return null;
            }

            // Generate all subsets of size k from uaAgents (these are the "movers")
            List<Set<Agent>> moverSubsets = chooseK(uaAgents, k);

            System.out.println("\n-------------------------------------------------------");
            System.out.println("[NUA_OD] Trying k=" + k + " (" + moverSubsets.size() + " subsets)");

            for (Set<Agent> movers : moverSubsets) {
                if (checkTimeout()) return null;

                // static = allUA \ movers
                Set<Agent> staticSet = new HashSet<>(uaAgents);
                staticSet.removeAll(movers);

                System.out.println("[OD] k=" + k + " | static=" + staticSet.size()
                        + " | dynamic=" + movers.size());

                long odStart = System.currentTimeMillis();

                AStarOD_Solver od = new AStartOD_Builder()
                        .setCostFunction(new SumServiceTimes())
                        .setTransientMAPFSettings(TransientMAPFSettings.defaultTransientMAPF)
                        .setStaticUAForUnassignedAgents(true)
                        .setStaticUA(staticSet)
                        .createAStarOD();

                // ── Metrics absorption ──
                InstanceReport odReport = new InstanceReport();
                odReport.keepSolutionString = false;

                long timeLeft = Math.max(
                        super.maximumRuntime - (System.nanoTime() / 1000000 - super.startTime), 0);

                RunParameters odParams = new RunParametersBuilder()
                        .setTimeout(timeLeft)
                        .setInstanceReport(odReport)
                        .createRP();

                Solution sol = od.solve(instance, odParams);
                absorbODMetrics(odReport);

                long odTime = System.currentTimeMillis() - odStart;
                this.odTotalTimeMS += odTime;

                if (sol != null) {
                    this.odMinNUA = k;

                    System.out.println("[OD] k=" + k + " → SOLUTION FOUND! (runtime: " + odTime + " ms)");
                    System.out.println("=======================================================");
                    System.out.println("[NUA_OD] Done. Minimum NUA = " + k);
                    System.out.println("=======================================================\n");

                    return sol;
                } else {
                    System.out.println("[OD] k=" + k + " → no solution with this subset. (runtime: " + odTime + " ms)");
                }
            }
        }

        System.out.println("[NUA_OD] No solution found for any k.");
        return null;
    }

    // ─────────────── Metrics ───────────────

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);
        instanceReport.putIntegerValue("OD_min_NUA", this.odMinNUA);
        instanceReport.putIntegerValue("OD_total_runtime", (int) this.odTotalTimeMS);
    }

    // ─────────────── Helpers ───────────────

    private List<Agent> getUAAgents(MAPF_Instance instance) {
        List<Agent> ua = new ArrayList<>();
        for (Agent a : instance.agents) {
            if (a.isUnassigned) ua.add(a);
        }
        return ua;
    }

    /**
     * Generate all subsets of size k from the given list.
     */
    private List<Set<Agent>> chooseK(List<Agent> agents, int k) {
        List<Set<Agent>> result = new ArrayList<>();
        chooseKHelper(agents, k, 0, new HashSet<>(), result);
        return result;
    }

    private void chooseKHelper(List<Agent> agents, int k, int start,
                               Set<Agent> current, List<Set<Agent>> result) {
        if (current.size() == k) {
            result.add(new HashSet<>(current));
            return;
        }
        if (start >= agents.size()) return;
        // remaining agents not enough to fill k
        if (agents.size() - start < k - current.size()) return;

        // include agents[start]
        current.add(agents.get(start));
        chooseKHelper(agents, k, start + 1, current, result);
        current.remove(agents.get(start));

        // exclude agents[start]
        chooseKHelper(agents, k, start + 1, current, result);
    }

    private void absorbODMetrics(InstanceReport odReport) {
        Integer expandedHL  = odReport.getIntegerValue(InstanceReport.StandardFields.expandedNodes);
        Integer generatedHL = odReport.getIntegerValue(InstanceReport.StandardFields.generatedNodes);

        if (expandedHL  != null) this.expandedNodes  += expandedHL;
        if (generatedHL != null) this.generatedNodes += generatedHL;

        Integer expandedLL  = odReport.getIntegerValue(InstanceReport.StandardFields.expandedNodesLowLevel);
        Integer generatedLL = odReport.getIntegerValue(InstanceReport.StandardFields.generatedNodesLowLevel);
        Integer llTime      = odReport.getIntegerValue(InstanceReport.StandardFields.totalLowLevelTimeMS);
        Integer llCalls     = odReport.getIntegerValue(InstanceReport.StandardFields.totalLowLevelCalls);

        if (expandedLL  != null) this.totalLowLevelNodesExpanded  += expandedLL;
        if (generatedLL != null) this.totalLowLevelNodesGenerated += generatedLL;
        if (llTime      != null) this.totalLowLevelTimeMS         += llTime;
        if (llCalls     != null) this.totalLowLevelCalls          += llCalls;
    }
}
