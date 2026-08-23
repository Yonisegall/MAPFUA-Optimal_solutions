package BasicMAPF.Solvers.CBS;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import BasicMAPF.CostFunctions.SumServiceTimes;
import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.A_Solver;
import BasicMAPF.Solvers.I_Solver;
import BasicMAPF.Solvers.LaCAM.LaCAMBuilder;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;

public class MinNUACBS_Solver extends A_Solver {

    @Override
    protected void init(MAPF_Instance instance, RunParameters parameters) {
        super.init(instance, parameters);

        this.name = "LaCAM_&_CBS_Dynamic_UA";
        
    }
    
// ────────────────────────────── Fields to store LaCAM metrics for CSV ──────────────────────────────

    private int lacamMinNUA      = -1;   // minimum k that LaCAM succeeded with
    private long lacamRuntimeMS  = -1;   // total LaCAM runtime across all k attempts
    private int lacamSolved      =  0;   // 1 if LaCAM found a feasible k, 0 otherwise
    private long cbsRuntimeMS    =  0;   // CBS runtime for the successful k (if LaCAM succeeded), or for the last attempted k (if LaCAM failed for all k)

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        List<Agent> uaAgents = getUAAgents(instance);

        // reset per-run metrics
        this.lacamMinNUA    = -1;
        this.lacamRuntimeMS = 0;
        this.lacamSolved    = 0;
        this.cbsRuntimeMS   = 0;

        System.out.println("=======================================================");
        System.out.println("[MinNUACBS] Starting. Total UA agents: " + uaAgents.size());
        System.out.println("=======================================================");

        for (int k = 0; k <= uaAgents.size(); k++) {
            if (checkTimeout()) {
                System.out.println("[MinNUACBS] TIMEOUT reached before finishing k=" + k);
                return null;
            }

            System.out.println("\n-------------------------------------------------------");
            System.out.println("[LaCAM] Trying with NUA budget k=" + k);

// ───────────────────────────────────────────── LaCAM ─────────────────────────────────────────────

            long lacamStart = System.currentTimeMillis();

            I_Solver lacamOracle = new LaCAMBuilder()
                    .setTransientMAPFBehaviour(TransientMAPFSettings.defaultTransientMAPF)
                    .setNUABudget(k)
                    .createLaCAM();

            Solution lacamSol = lacamOracle.solve(instance, parameters);

            long lacamIterTime = System.currentTimeMillis() - lacamStart;
            this.lacamRuntimeMS += lacamIterTime;   // accumulate across all k attempts

            if (lacamSol == null) {
                System.out.println("[LaCAM] k=" + k + " → NO solution. (iter runtime: " + lacamIterTime + " ms)");
                continue;
            }

            // LaCAM succeeded
            Set<Agent> movedUA = identifyMovedUA(lacamSol, instance);
            this.lacamSolved  = 1;
            this.lacamMinNUA  = movedUA.size();   // actual number of UA that moved (≤ k)

            System.out.println("[LaCAM] k=" + k + " → FOUND! Moved UA: " + movedUA.size() +
                    " " + movedUA + " (iter runtime: " + lacamIterTime + " ms)" +
                    " | Total LaCAM time so far: " + this.lacamRuntimeMS + " ms");

// ───────────────────────────────────────────── CBS ─────────────────────────────────────────────

            Set<Agent> staticSet = getStaticSet(uaAgents, movedUA);
            System.out.println("[CBS]   k=" + k + " → Starting CBS | static UA: " + staticSet.size() +
                    " | dynamic UA: " + movedUA.size());

            long cbsStart = System.currentTimeMillis();

            I_Solver cbsSolver = new CBSBuilder()
                    .setCostFunction(new SumServiceTimes())
                    .setTransientMAPFSettings(TransientMAPFSettings.defaultTransientMAPF)
                    .setStaticUAForUnassignedAgents(true)
                    .setStaticUA(staticSet)
                    .createCBS_Solver();

            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // Solution finalSol = cbsSolver.solve(instance, parameters);

            InstanceReport cbsReport = new InstanceReport();
            cbsReport.keepSolutionString = false;

            long timeLeftToTimeout = Math.max(super.maximumRuntime - (System.nanoTime()/1000000 - super.startTime), 0);

            RunParameters cbsParams = new RunParametersBuilder()
                    .setTimeout(timeLeftToTimeout)
                    .setConstraints(parameters.constraints)           // keep same constraints (could be null)
                    .setExistingSolution(parameters.existingSolution) // keep same existingSolution (could be null)
                    .setAStarGAndH(parameters.singleAgentGAndH)       // keep same heuristic (could be null)
                    .setInstanceReport(cbsReport)
                    .createRP();

            Solution finalSol = cbsSolver.solve(instance, cbsParams);

            absorbCBSMetrics(cbsReport);
            ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


            long cbsTime = System.currentTimeMillis() - cbsStart;

            this.cbsRuntimeMS = cbsTime;

            if (finalSol == null) {
                System.out.println("[CBS]   k=" + k + " → FAILED (timeout). (runtime: " + cbsTime + " ms)");
                System.out.println("[CBS]   WARNING: LaCAM confirmed k=" + k +
                        " is feasible but CBS timed out. Returning null.");
                return null;
            }

            int makespan = 0;
            for (SingleAgentPlan p : finalSol) {                
                if (!p.agent.isUnassigned && p.getEndTime() > makespan) makespan = p.getEndTime();
            }
  
            // Find the static agent according
            Set<Integer> staticIDs = new HashSet<>();

            for (Agent ua : staticSet) staticIDs.add(ua.iD);

            for (Agent ua : instance.agents) {
                if (staticIDs.contains(ua.iD) && finalSol.getPlanFor(ua) == null) {
                    SingleAgentPlan stayPlan = new SingleAgentPlan(ua);
                    I_Location loc = instance.map.getMapLocation(ua.source);
                    for (int t = 1; t <= Math.max(makespan, 1); t++) {
                        stayPlan.addMove(new Move(ua, t, loc, loc));
                    }
                    finalSol.putPlan(stayPlan);
                }
            }

            for (Agent ua : instance.agents) {
                if (!ua.isUnassigned) continue;          // Only UA
                if (staticIDs.contains(ua.iD)) continue; // Only dynamic

                SingleAgentPlan plan = finalSol.getPlanFor(ua);

                if (plan == null) continue;
                if (plan.getEndTime() >= makespan) continue;

                I_Location lastLoc = plan.getLastMove().currLocation;

                for (int t = plan.getEndTime() + 1; t <= makespan; t++) 
                    plan.addMove(new Move(ua, t, lastLoc, lastLoc));
            }

            System.out.println("[CBS]   k=" + k + " → OPTIMAL solution FOUND! (runtime: " + cbsTime + " ms)");
            System.out.println("=======================================================");
            System.out.println("[MinNUACBS] Done. Minimum moving UA = " + movedUA.size());
            System.out.println("=======================================================\n");

            System.out.println("[DEBUG] finalSol agents before remap:");

            for (SingleAgentPlan p : finalSol) {
                System.out.println("  Agent ID=" + p.agent.iD + 
                    " source=" + p.agent.source + 
                    " target=" + p.agent.target +
                    " planEnd=" + p.getEndTime());
            }

            System.out.println("[DEBUG] instance.agents:");

            for (Agent a : instance.agents) {
                System.out.println("  Agent ID=" + a.iD + 
                    " source=" + a.source + 
                    " target=" + a.target);
            }
            return remapSolutionToOriginal(finalSol, instance);
        }

        System.out.println("[MinNUACBS] No solution found for any k up to " + uaAgents.size());

        return null;
    }

// ─────────────────────── Write LaCAM metrics into the instance report (→ CSV) ───────────────────────

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);   // writes all the standard CBS/common fields

        // These three fields are ONLY written by MinNUACBS_Solver
        instanceReport.putIntegerValue("LaCAM_solved",    this.lacamSolved);
        instanceReport.putIntegerValue("LaCAM_min_NUA",    this.lacamMinNUA);
        instanceReport.putIntegerValue("LaCAM_runtime", (int) this.lacamRuntimeMS);
        instanceReport.putIntegerValue("CBS_runtime",   (int) this.cbsRuntimeMS);
    }

// ───────────────────────────────────────────── Helpers ─────────────────────────────────────────────

    private Set<Agent> identifyMovedUA(Solution sol, MAPF_Instance instance) {
        Set<Agent> moved = new HashSet<>();
        for (SingleAgentPlan plan : sol) {
            if (plan.agent.isUnassigned) {
                I_Location source = instance.map.getMapLocation(plan.agent.source);
                System.out.println("[DEBUG identifyMovedUA] Agent " + plan.agent.iD + 
                    " source=" + plan.agent.source);
                for (Move m : plan) {
                    System.out.println("  t=" + m.timeNow + 
                        " prev=" + m.prevLocation + 
                        " curr=" + m.currLocation + 
                        " equals_source=" + m.currLocation.equals(source));
                    if (!m.currLocation.equals(source)) {
                        moved.add(plan.agent);
                        break;
                    }
                }
            }
        }
        return moved;
    }

    private Set<Agent> getStaticSet(List<Agent> allUA, Set<Agent> movedUA) {
        Set<Agent> staticUA = new HashSet<>(allUA);
        staticUA.removeAll(movedUA);
        return staticUA;
    }

    private List<Agent> getUAAgents(MAPF_Instance instance) {
        List<Agent> ua = new ArrayList<>();
        for (Agent a : instance.agents) {
            if (a.isUnassigned) ua.add(a);
        }
        return ua;
    }

    private Solution remapSolutionToOriginal(Solution sol, MAPF_Instance original) {
        Map<Integer, Agent> idToOrig = new HashMap<>();
        for (Agent a : original.agents) idToOrig.put(a.iD, a);

        Solution mapped = new Solution();

        for (Agent orig : original.agents) {
            SingleAgentPlan foundPlan = null;

            for (SingleAgentPlan plan : sol) {
                if (plan.agent.iD == orig.iD) {
                    foundPlan = plan;
                    break;
                }
            }

            if (foundPlan == null) continue;

            ArrayList<Move> newMoves = new ArrayList<>();

            for (Move mv : foundPlan) {
                newMoves.add(new Move(orig, mv.timeNow, mv.prevLocation, mv.currLocation));
            }
            mapped.putPlan(new SingleAgentPlan(orig, newMoves));
        }
        return mapped;
    }

    private void absorbCBSMetrics(InstanceReport cbsReport) {

        // High‑level CBS tree
        Integer expandedHL = cbsReport.getIntegerValue("Expanded Nodes (High Level)");
        Integer generatedHL = cbsReport.getIntegerValue("Generated Nodes (High Level)");

        if (expandedHL != null) this.expandedNodes += expandedHL;
        if (generatedHL != null) this.generatedNodes += generatedHL;

        // Low‑level A* (agents replanning)
        Integer expandedLL = cbsReport.getIntegerValue("Expanded Nodes (Low Level)");
        Integer generatedLL = cbsReport.getIntegerValue("Generated Nodes (Low Level)");
        Integer llTime = cbsReport.getIntegerValue("Total Low Level Time (ms)");
        Integer llCalls = cbsReport.getIntegerValue("Total Calls to Low Level");

        if (expandedLL != null) this.totalLowLevelNodesExpanded += expandedLL;
        if (generatedLL != null) this.totalLowLevelNodesGenerated += generatedLL;
        if (llTime != null) this.totalLowLevelTimeMS += llTime;
        if (llCalls != null) this.totalLowLevelCalls += llCalls;
    }
}




    ////////////////////////////////////// LaCAM Only //////////////////////////////////////
    
    // @Override
    // protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
    //     List<Agent> uaAgents = getUAAgents(instance);

    //     if (uaAgents.isEmpty()) {
    //         I_Solver lacamOracle = new LaCAMBuilder()
    //                 .setTransientMAPFBehaviour(TransientMAPFSettings.defaultTransientMAPF)
    //                 .createLaCAM();

    //         return lacamOracle.solve(instance, parameters);
    //     }

    //     for (int k = uaAgents.size(); k >= 0; k--) {
    //         List<Set<Agent>> subsets = chooseK(uaAgents, k);

    //         for (Set<Agent> staticUA : subsets) {
    //             if (checkTimeout()) return null;

    //             System.out.println("STATIC UA: " + staticUA);

    //             I_Solver lacamOracle = new LaCAMBuilder()
    //                     .setTransientMAPFBehaviour(TransientMAPFSettings.defaultTransientMAPF)
    //                     .setStaticUAForUnassignedAgents(true)
    //                     .setStaticUA(staticUA)
    //                     .createLaCAM();

    //             Solution sol = lacamOracle.solve(instance, parameters);

    //             if (sol != null) {
    //                 return remapSolutionToOriginal(sol, instance);
    //             }
    //         }
    //     }

    //     return null;
    // }








    ////////////////////////////////////// CBS Only ///////////////////////////////////////////////

    // @Override
    // protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
    //     List<Agent> uaAgents = getUAAgents(instance);

    //     if (uaAgents.isEmpty()) {
    //         I_Solver solver = new CBSBuilder()
    //                 .setCostFunction(new SumServiceTimes())
    //                 .setTransientMAPFSettings(TransientMAPFSettings.defaultTransientMAPF)
    //                 .createCBS_Solver();

    //         return solver.solve(instance, parameters);
    //     }

    //     for (int k = uaAgents.size(); k >= 0; k--) {

    //         List<Set<Agent>> subsets = chooseK(uaAgents, k);

    //         Solution bestForThisK = null;
    //         int bestFuel = Integer.MAX_VALUE;
    //         int bestSST = Integer.MAX_VALUE;

    //         for (Set<Agent> staticUA : subsets) {

    //             if (checkTimeout()) return null;

    //             System.out.println("STATIC UA: " + staticUA);

    //             I_Solver solver = new CBSBuilder()
    //                     .setCostFunction(new SumServiceTimes())
    //                     .setTransientMAPFSettings(TransientMAPFSettings.defaultTransientMAPF)
    //                     .setStaticUAForUnassignedAgents(true)
    //                     .setStaticUA(staticUA)  
    //                     .createCBS_Solver();

    //             Solution sol = solver.solve(instance, parameters);

    //             if (sol == null) continue;

    //             sol = remapSolutionToOriginal(sol, instance);

    //             int fuel = sol.FUEL();
    //             int sst = sol.SST();

    //             if (bestForThisK == null ||
    //                     sst < bestSST ||
    //                     (sst == bestSST && fuel < bestFuel)) {

    //                 bestForThisK = sol;
    //                 bestFuel = fuel;
    //                 bestSST = sst;
    //             }
    //         }
    //         if (bestForThisK != null) {
    //             return bestForThisK;
    //         }
    //     }

    //     return null;
    // }
