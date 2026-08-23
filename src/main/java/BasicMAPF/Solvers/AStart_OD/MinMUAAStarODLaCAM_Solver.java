package BasicMAPF.Solvers.AStart_OD;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import BasicMAPF.CostFunctions.SumServiceTimes;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Solvers.A_Solver;
import BasicMAPF.Solvers.I_Solver;
import BasicMAPF.Solvers.LaCAM.LaCAMBuilder;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;


public class MinMUAAStarODLaCAM_Solver extends A_Solver {

    private int lacamMinMUA     =-1;
    private long lacamRuntimeMS = 0;
    private int lacamSolved     = 0;
    private long odRuntimeMS    = 0;

    @Override
    protected void init(MAPF_Instance instance, RunParameters parameters) {
        super.init(instance, parameters);
        this.name = "LaCAM_&_OD_Dynamic_UA";
        this.setConfiguredCostFunctionName("MUA");
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {

        List<Agent> uaAgents = getUAAgents(instance);

        // reset per-run metrics
        this.lacamMinMUA    =-1;
        this.lacamRuntimeMS = 0;
        this.lacamSolved    = 0;
        this.odRuntimeMS    = 0;

        System.out.println("=======================================================");
        System.out.println("[MinMUAOD] Starting. Total UA agents: " + uaAgents.size());
        System.out.println("=======================================================");

        for (int k = 0; k <= uaAgents.size(); k++) {
            if (checkTimeout()) {
                System.out.println("[MinMUAOD] TIMEOUT reached before finishing k=" + k);
                return null;
            }

            System.out.println("\n-------------------------------------------------------");
            System.out.println("[LaCAM] Trying with MUA budget k=" + k);

            // ─────────────── LaCAM ───────────────
            long lacamStart = System.currentTimeMillis();
            I_Solver lacam = new LaCAMBuilder()
                    .setTransientMAPFBehaviour(TransientMAPFSettings.defaultTransientMAPF)
                    .setMUABudget(k)
                    .createLaCAM();
            Solution lacamSol = lacam.solve(instance, parameters);
            long lacamIterTime = System.currentTimeMillis() - lacamStart;
            this.lacamRuntimeMS += lacamIterTime;

            if (lacamSol == null) {
                System.out.println("[LaCAM] k=" + k + " → NO solution. (iter runtime: " + lacamIterTime + " ms)");
                continue;
            }

            Set<Integer> movedUAIds = identifyMovedUAIds(lacamSol, instance);
            this.lacamSolved = 1;
            this.lacamMinMUA = movedUAIds.size();

            System.out.println("[LaCAM] k=" + k + " → FOUND! Moved UA: " + movedUAIds.size()
                    + " (iter runtime: " + lacamIterTime + " ms)"
                    + " | Total LaCAM time so far: " + this.lacamRuntimeMS + " ms");

            // ─────────────── OD ───────────────
            // static = allUA \ moved
            Set<Agent> staticSet = new HashSet<>();
            for (Agent a : uaAgents) {
                if (!movedUAIds.contains(a.iD)) staticSet.add(a);
            }

            System.out.println("[OD]    k=" + k + " → Starting OD | static UA: " + staticSet.size()
                    + " | dynamic UA: " + movedUAIds.size());

            long odStart = System.currentTimeMillis();

            AStarOD_Solver od = new AStartOD_Builder()
                    .setCostFunction(new SumServiceTimes())
                    .setTransientMAPFSettings(TransientMAPFSettings.defaultTransientMAPF)
                    .setStaticUAForUnassignedAgents(true)
                    .setStaticUA(staticSet)
                    .createAStarOD();

            // ── Metrics absorption (like MinMUACBS) ──
            Environment.Metrics.InstanceReport odReport = new Environment.Metrics.InstanceReport();
            odReport.keepSolutionString = false;

            long timeLeft = Math.max(super.maximumRuntime - (System.nanoTime()/1000000 - super.startTime), 0);

            RunParameters odParams = new BasicMAPF.DataTypesAndStructures.RunParametersBuilder()
                    .setTimeout(timeLeft)
                    .setInstanceReport(odReport)
                    .createRP();

            Solution sol = od.solve(instance, odParams);
            absorbODMetrics(odReport);

            long odTime = System.currentTimeMillis() - odStart;
            this.odRuntimeMS = odTime;

            if (sol == null) {
                System.out.println("[OD]    k=" + k + " → FAILED. (runtime: " + odTime + " ms)");
                continue;
            }

            System.out.println("[OD]    k=" + k + " → OPTIMAL solution FOUND! (runtime: " + odTime + " ms)");
            System.out.println("=======================================================");
            System.out.println("[MinMUAOD] Done. Minimum moving UA = " + movedUAIds.size());
            System.out.println("=======================================================\n");

            return sol;
        }

        System.out.println("[MinMUAOD] No solution found for any k up to " + uaAgents.size());
        return null;
    }

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);
        instanceReport.putIntegerValue("LaCAM_solved", lacamSolved);
        instanceReport.putIntegerValue("LaCAM_min_MUA", lacamMinMUA);
        instanceReport.putIntegerValue("LaCAM_runtime", (int) lacamRuntimeMS);
        instanceReport.putIntegerValue("OD_runtime", (int) odRuntimeMS);
    }

    private List<Agent> getUAAgents(MAPF_Instance instance) {
        List<Agent> ua = new ArrayList<>();
        for (Agent a : instance.agents) if (a.isUnassigned) ua.add(a);
        return ua;
    }

    private Set<Integer> identifyMovedUAIds(Solution sol, MAPF_Instance instance) {
        Set<Integer> moved = new HashSet<>();
        for (var plan : sol) {
            Agent a = plan.agent;
            if (!a.isUnassigned) continue;
            var source = instance.map.getMapLocation(a.source);
            for (var mv : plan) {
                if (!mv.currLocation.equals(source)) { moved.add(a.iD); break; }
            }
        }
        return moved;
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
