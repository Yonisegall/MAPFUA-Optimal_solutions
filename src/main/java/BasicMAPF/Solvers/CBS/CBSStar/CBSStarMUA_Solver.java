package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.Coordinates.I_Coordinate;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.AStart_OD.MAPFUA_MA.MapfuaObjective;
import BasicMAPF.Solvers.A_Solver;
import BasicMAPF.Solvers.ConstraintsAndConflicts.A_Conflict;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;
import TransientMAPF.TransientMAPFSolution;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * CBS* for MAPFUA with one selected pure high-level objective.
 *
 * Source-algorithm mapping:
 * root singleton meta-agents, DetectJointLoop before conflict validation on non-root nodes, loop-triggered merging of
 * all repeating meta-agents, branch-specific constrained joint replanning, and fallback unconstrained joint replanning
 * follow Algorithms 1-3 in the supplied CBS* paper/appendix. The MAPFUA-specific adaptation is that loop signatures
 * and low-level duplicate keys retain transient AA target-visit history and UA moved history, and the low level does
 * not impose a terminal target on UAs.
 */
public class CBSStarMUA_Solver extends A_Solver {
    private final MapfuaObjective objective;
    private PriorityQueue<CBSStarNode> open;
    private long nextNodeId;

    public CBSStarMUA_Solver() {
        this(MapfuaObjective.MUA);
    }

    public CBSStarMUA_Solver(MapfuaObjective objective) {
        this.objective = objective;
        this.name = switch (objective) {
            case MUA -> "CBSStarMUA";
            case FUEL -> "CBSStarFuel";
            case SST -> "CBSStarSST";
        };
        this.configuredCostFunctionName = objective.reportName;
        this.transientMAPFSettings = TransientMAPFSettings.defaultTransientMAPF;
    }

    @Override
    protected void init(MAPF_Instance instance, RunParameters parameters) {
        super.init(instance, parameters);
        this.transientMAPFSettings = TransientMAPFSettings.defaultTransientMAPF;
        this.open = new PriorityQueue<>(Comparator
                .comparingInt((CBSStarNode node) -> node.cost)
                .thenComparingLong(node -> node.id));
        this.nextNodeId = 0;
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        CBSStarNode root = generateRoot(instance, parameters);
        if (root == null) return null;
        open.add(root);
        generatedNodes++;

        while (!open.isEmpty()) {
            if (checkTimeout()) return null;
            CBSStarNode node = open.poll();
            expandedNodes++;

            Set<MetaAgent> loop = node.root ? Set.of() : detectJointLoop(node);
            if (!loop.isEmpty()) {
                handleLoopMerge(node, loop, parameters);
                continue;
            }

            A_Conflict conflict = node.solution.firstConflict(false, false);
            if (conflict == null) {
                Solution finalSolution = normalizeFinalSolution(node.solution);
                if (finalSolution.firstConflict(false, false) != null) {
                    continue;
                }
                runtimeToFirstSolution = (int) (System.currentTimeMillis() - startDate);
                return finalSolution;
            }

            for (Constraint constraint : conflict.getPreventingConstraints()) {
                addConflictChild(node, constraint, parameters);
            }
        }
        return null;
    }

    private CBSStarNode generateRoot(MAPF_Instance instance, RunParameters parameters) {
        MetaAgentPartition partition = MetaAgentPartition.singletonPartition(instance.agents);
        ConstraintSet rootConstraints = new ConstraintSet();
        if (parameters.constraints != null) rootConstraints.addAll(parameters.constraints);
        Solution solution = new Solution();
        MetaAgent newest = null;
        for (MetaAgent metaAgent : partition.metaAgents()) {
            Solution groupSolution = solveMetaAgent(metaAgent, rootConstraints);
            if (groupSolution == null) return null;
            putAll(solution, groupSolution);
            newest = metaAgent;
        }
        return new CBSStarNode(rootConstraints, solution, partition, newest, objectiveValue(solution), 0, nextNodeId++, true);
    }

    private void addConflictChild(CBSStarNode parent, Constraint constraint, RunParameters parameters) {
        MetaAgent replanned = parent.partition.containing(constraint.agent);
        ConstraintSet childConstraints = new ConstraintSet(parent.constraints);
        childConstraints.add(constraint);
        Solution childSolution = new Solution(parent.solution);
        Solution groupSolution = solveMetaAgent(replanned, childConstraints);
        if (groupSolution == null) return;
        putAll(childSolution, groupSolution);
        CBSStarNode child = parent.childWith(constraint, childSolution, replanned,
                objectiveValue(childSolution), nextNodeId++);
        open.add(child);
        generatedNodes++;
    }

    private void handleLoopMerge(CBSStarNode node, Set<MetaAgent> loop, RunParameters parameters) {
        MetaAgentPartition mergedPartition = node.partition.merge(loop);
        MetaAgent merged = mergedPartition.containing(loop.iterator().next().agents().get(0));
        ConstraintSet relevant = relevantConstraints(node.constraints, merged.agents());
        Solution groupSolution = solveMetaAgent(merged, relevant);
        if (groupSolution == null) {
            if (relevant.isEmpty()) {
                open.clear();
                return;
            }
            Solution unconstrained = solveMetaAgent(merged, new ConstraintSet());
            if (unconstrained == null) {
                open.clear();
            }
            return;
        }

        Solution mergedSolution = new Solution(node.solution);
        for (MetaAgent old : loop) {
            for (Agent agent : old.agents()) {
                SingleAgentPlan replacement = groupSolution.getPlanFor(agent);
                if (replacement == null) {
                    throw new IllegalStateException("Merged low-level did not return a plan for " + agent);
                }
                mergedSolution.putPlan(replacement);
            }
        }
        if (!internalConflictFree(groupSolution)) {
            throw new IllegalStateException("CBS* joint low-level returned an internally conflicting meta-agent path.");
        }
        CBSStarNode child = node.withMerge(mergedPartition, merged, mergedSolution,
                objectiveValue(mergedSolution), nextNodeId++);
        open.add(child);
        generatedNodes++;
    }

    private Solution solveMetaAgent(MetaAgent metaAgent, ConstraintSet constraints) {
        long lowLevelStart = System.currentTimeMillis();
        CBSStarJointLowLevel lowLevel = new CBSStarJointLowLevel(instance, transientMAPFSettings, getTimeout(), objective);
        Solution solution = lowLevel.solve(metaAgent, constraints);
        totalLowLevelCalls++;
        totalLowLevelNodesGenerated += lowLevel.generated();
        totalLowLevelNodesExpanded += lowLevel.expanded();
        totalLowLevelTimeMS += (int) (System.currentTimeMillis() - lowLevelStart);
        return solution;
    }

    private ConstraintSet relevantConstraints(ConstraintSet constraints, Collection<Agent> agents) {
        ConstraintSet relevant = new ConstraintSet();
        for (ArrayList<Constraint> atLocation : constraints.getLocationConstraintsTimeSorted().values()) {
            for (Constraint constraint : atLocation) {
                if (constraint.agent == null || agents.contains(constraint.agent)) relevant.add(constraint);
            }
        }
        for (Constraint constraint : constraints.getGoalConstraints().values()) {
            if (constraint.agent == null || agents.contains(constraint.agent)) relevant.add(constraint);
        }
        return relevant;
    }

    private void putAll(Solution target, Solution source) {
        for (SingleAgentPlan plan : source) {
            target.putPlan(plan);
        }
    }

    private boolean internalConflictFree(Solution solution) {
        return solution.firstConflict(false, false) == null;
    }

    private Set<MetaAgent> detectJointLoop(CBSStarNode node) {
        MetaAgent newest = node.newestMetaAgent;
        int horizon = node.solution.endTime();
        for (int t1 = 0; t1 < horizon; t1++) {
            LoopSignature newestAtT1 = signature(node.solution, newest, t1);
            for (int t2 = t1 + 1; t2 <= horizon; t2++) {
                if (!newestAtT1.equals(signature(node.solution, newest, t2))) continue;
                Set<MetaAgent> repeating = new HashSet<>();
                for (MetaAgent metaAgent : node.partition.metaAgents()) {
                    if (signature(node.solution, metaAgent, t1).equals(signature(node.solution, metaAgent, t2))) {
                        repeating.add(metaAgent);
                    }
                }
                if (repeating.size() >= 2 && nonTerminalRepeat(node.solution, repeating, t1, t2)) {
                    return repeating;
                }
            }
        }
        return Set.of();
    }

    private boolean nonTerminalRepeat(Solution solution, Set<MetaAgent> repeating, int t1, int t2) {
        for (MetaAgent metaAgent : repeating) {
            for (Agent agent : metaAgent.agents()) {
                for (int t = t1; t < t2; t++) {
                    I_Location now = locationAt(solution, agent, t);
                    I_Location next = locationAt(solution, agent, t + 1);
                    if (!now.equals(next)) return true;
                    if (!agent.isUnassigned && !now.getCoordinate().equals(agent.target)) return true;
                }
            }
        }
        return false;
    }

    private LoopSignature signature(Solution solution, MetaAgent metaAgent, int time) {
        List<I_Coordinate> locations = new ArrayList<>();
        List<Boolean> visitedTargets = new ArrayList<>();
        List<Boolean> movedUAs = new ArrayList<>();
        for (Agent agent : metaAgent.agents()) {
            locations.add(locationAt(solution, agent, time).getCoordinate());
            visitedTargets.add(visitedTargetBy(solution.getPlanFor(agent), time));
            movedUAs.add(movedUABy(solution.getPlanFor(agent), time));
        }
        return new LoopSignature(locations, visitedTargets, movedUAs);
    }

    private I_Location locationAt(Solution solution, Agent agent, int time) {
        SingleAgentPlan plan = solution.getPlanFor(agent);
        if (time == 0) return instance.map.getMapLocation(agent.source);
        if (plan.getEndTime() < time) return plan.getLastMove().currLocation;
        Move move = plan.moveAt(time);
        return move == null ? instance.map.getMapLocation(agent.source) : move.currLocation;
    }

    private boolean visitedTargetBy(SingleAgentPlan plan, int time) {
        Agent agent = plan.agent;
        if (agent.isUnassigned) return true;
        if (plan.getFirstMoveTime() > time) return agent.source.equals(agent.target);
        for (Move move : plan) {
            if (move.timeNow > time) break;
            if (move.prevLocation.getCoordinate().equals(agent.target) || move.currLocation.getCoordinate().equals(agent.target)) {
                return true;
            }
        }
        return false;
    }

    private boolean movedUABy(SingleAgentPlan plan, int time) {
        if (!plan.agent.isUnassigned) return false;
        for (Move move : plan) {
            if (move.timeNow > time) break;
            if (!move.prevLocation.equals(move.currLocation)) return true;
        }
        return false;
    }

    private Solution normalizeFinalSolution(Solution solution) {
        Solution normalized = new TransientMAPFSolution();
        for (SingleAgentPlan plan : solution) {
            if (plan.agent.isUnassigned) {
                normalized.putPlan(trimTrailingWaits(plan));
            } else {
                normalized.putPlan(trimAfterFirstTargetVisit(plan));
            }
        }
        return normalized;
    }

    private SingleAgentPlan trimAfterFirstTargetVisit(SingleAgentPlan plan) {
        ArrayList<Move> moves = new ArrayList<>();
        for (Move move : plan) {
            moves.add(move);
            if (move.currLocation.getCoordinate().equals(plan.agent.target)) {
                break;
            }
        }
        return new SingleAgentPlan(plan.agent, moves);
    }

    private SingleAgentPlan trimTrailingWaits(SingleAgentPlan plan) {
        ArrayList<Move> moves = new ArrayList<>();
        for (Move move : plan) moves.add(move);
        while (moves.size() > 1) {
            Move last = moves.get(moves.size() - 1);
            if (!last.prevLocation.equals(last.currLocation)) break;
            moves.remove(moves.size() - 1);
        }
        return new SingleAgentPlan(plan.agent, moves);
    }

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);
        if (instanceReport != null && solution != null) {
            instanceReport.putFloatValue(InstanceReport.StandardFields.solutionCost, objectiveValue(solution));
            instanceReport.putStringValue(InstanceReport.StandardFields.solutionCostFunction, objective.reportName);
        }
    }

    int objectiveValue(Solution solution) {
        return switch (objective) {
            case MUA -> solution.MUA();
            case FUEL -> solution.FUEL();
            case SST -> solution.SST();
        };
    }

    @Override
    protected void releaseMemory() {
        super.releaseMemory();
        this.open = null;
    }

    private record LoopSignature(List<I_Coordinate> locations, List<Boolean> visitedTargets, List<Boolean> movedUAs) {
    }
}
