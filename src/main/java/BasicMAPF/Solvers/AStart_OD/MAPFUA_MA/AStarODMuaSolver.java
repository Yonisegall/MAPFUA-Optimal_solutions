package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.Enum_MapLocationType;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.A_Solver;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;
import TransientMAPF.TransientMAPFSolution;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;

/**
 * Optimal MAPFUA-MA A*OD solver from thesis Chapter 4, adapted to this
 * repository's transient MAPFUA mode. The same search engine is configured
 * with one primary objective: pure MUA, pure Fuel, or pure SST. No secondary
 * objective participates in OPEN ordering or duplicate dominance; equal f
 * values are broken only by deterministic node id. The duplicate key keeps OD
 * phase, partial move prefix, current positions, movedUAs, and visitedTargets,
 * so states with different objective or transient histories are not merged.
 */
public class AStarODMuaSolver extends A_Solver {
    private final MapfuaObjective objective;
    private final MuaHeuristicType heuristicType;
    private final boolean lazyEvaluation;
    private final TransientMAPFSettings configuredTransientSettings;
    private MuaProblemContext context;
    private MuaHeuristic heuristic;
    private PriorityQueue<MuaSearchState> open;
    private Map<MuaSearchState.MuaStateKey, Long> bestG;
    private long serial;
    private long intermediateNodes;
    private long fullNodes;
    private long rootH;

    public AStarODMuaSolver(MapfuaObjective objective, MuaHeuristicType heuristicType, boolean lazyEvaluation,
                            TransientMAPFSettings transientMAPFSettings) {
        this.objective = Objects.requireNonNull(objective);
        this.heuristicType = Objects.requireNonNull(heuristicType);
        this.lazyEvaluation = lazyEvaluation;
        this.configuredTransientSettings = Objects.requireNonNullElse(transientMAPFSettings, TransientMAPFSettings.defaultTransientMAPF);
        this.transientMAPFSettings = this.configuredTransientSettings;
        this.name = solverName(objective, heuristicType);
        this.setConfiguredCostFunctionName(objective.reportName);
    }

    public static String solverName(MapfuaObjective objective, MuaHeuristicType type) {
        String objectiveName = switch (objective) {
            case MUA -> "MUA";
            case FUEL -> "Fuel";
            case SST -> "SST";
        };
        String heuristicName = switch (type) {
            case ZERO -> "Zero";
            case MAX -> "Max";
            case INCREASING -> "IH";
            case MERGE -> "MH";
        };
        return "AStarOD" + objectiveName + heuristicName;
    }

    @Override
    protected void init(MAPF_Instance instance, RunParameters parameters) {
        super.init(instance, parameters);
        this.transientMAPFSettings = configuredTransientSettings;
        this.context = new MuaProblemContext(instance);
        this.heuristic = new MapfuaCaptureHeuristic(context, objective, heuristicType, lazyEvaluation);
        this.open = new PriorityQueue<>(nodeComparator);
        this.bestG = new HashMap<>();
        this.serial = 0;
        this.generatedNodes = 0;
        this.expandedNodes = 0;
        this.intermediateNodes = 0;
        this.fullNodes = 0;
        this.rootH = 0;
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        MuaSearchState root = root();
        rootH = heuristic.estimate(root);
        root = withHeuristic(root, rootH);
        pushIfBetter(root);
        while (!open.isEmpty()) {
            if (checkTimeout()) return null;
            MuaSearchState current = open.poll();
            if (isStale(current)) continue;
            if (!current.partial && isGoal(current)) return buildSolution(current);
            expandedNodes++;
            if (current.partial) {
                intermediateNodes++;
                expandPartial(current);
            } else {
                fullNodes++;
                expandFull(current);
            }
        }
        return null;
    }

    /**
     * Accumulated search cost of a node. A full node's g is its real cost. An OD partial node stores its base
     * (full) state's g; under MUA the first moves of UAs already chosen in its prefix are irrevocable, so they are
     * charged here immediately (stepNewUA) instead of only when the joint step completes. This is standard OD cost
     * decomposition: the completed child's g (base.g + stepNewUA) is unchanged. Fuel and SST keep the stored g.
     */
    long effectiveG(MuaSearchState node) {
        return node.partial && objective == MapfuaObjective.MUA ? node.g + node.stepNewUA : node.g;
    }

    /** A polled node is stale if a cheaper copy of the same state was pushed after it. */
    boolean isStale(MuaSearchState node) {
        return !Objects.equals(bestG.get(node.key()), effectiveG(node));
    }

    /** Test seam: current OPEN contents (unordered). */
    List<MuaSearchState> openSnapshot() {
        return new ArrayList<>(open);
    }

    MuaSearchState root() {
        int n = context.agents.size();
        I_Location[] positions = new I_Location[n];
        BitSet moved = new BitSet(context.uaStartLocations.length);
        BitSet visited = new BitSet(n);
        for (int i = 0; i < n; i++) {
            Agent agent = context.agents.get(i);
            positions[i] = context.map.getMapLocation(agent.source);
            if (agent.isUnassigned || positions[i].getCoordinate().equals(agent.target)) visited.set(i);
        }
        return MuaSearchState.full(0, positions, moved, visited, 0, 0, null, null, nextId());
    }

    private MuaSearchState withHeuristic(MuaSearchState node, long h) {
        if (node.partial) {
            return MuaSearchState.partial(node.parent, node.nextAgent, node.nextPositions, node.nextMovedUAs,
                    node.nextVisitedTargets, node.chosenMoves, h, node.id, node.stepFuel, node.stepNewUA);
        }
        return MuaSearchState.full(node.time, node.positions, node.movedUAs, node.visitedTargets,
                node.g, h, node.parent, node.jointMovesFromParent, node.id);
    }

    private boolean isGoal(MuaSearchState node) {
        if (transientMAPFSettings.isTransientMAPF()) {
            for (int agentIndex : context.assignedIndexes) {
                if (!node.visitedTargets.get(agentIndex)) return false;
            }
            return true;
        }
        for (int agentIndex : context.assignedIndexes) {
            if (!node.positions[agentIndex].getCoordinate().equals(context.agents.get(agentIndex).target)) return false;
        }
        return true;
    }

    void expandFull(MuaSearchState node) {
        int n = context.agents.size();
        expandAgent(node, 0, new I_Location[n], (BitSet) node.movedUAs.clone(),
                (BitSet) node.visitedTargets.clone(), new Move[n], 0, 0);
    }

    void expandPartial(MuaSearchState node) {
        expandAgent(node.parent, node.nextAgent, node.nextPositions, node.nextMovedUAs,
                node.nextVisitedTargets, node.chosenMoves, node.stepFuel, node.stepNewUA);
    }

    private void expandAgent(MuaSearchState base, int agentIndex, I_Location[] nextPositions,
                             BitSet movedUAs, BitSet visitedTargets, Move[] chosenMoves,
                             int stepFuel, int stepNewUA) {
        int n = context.agents.size();
        if (agentIndex == n) {
            long g = Math.addExact(base.g, objective.stepCost(stepNewUA, stepFuel, stepSST(base)));
            MuaSearchState child = MuaSearchState.full(base.time + 1, Arrays.copyOf(nextPositions, n),
                    movedUAs, visitedTargets, g, 0, base, Arrays.copyOf(chosenMoves, n), nextId());
            pushIfBetter(withHeuristic(child, heuristic.estimate(child)));
            return;
        }
        Agent agent = context.agents.get(agentIndex);
        I_Location current = base.positions[agentIndex];
        for (I_Location dst : legalMoves(current)) {
            Move move = new Move(agent, base.time + 1, current, dst);
            if (conflictsWithPrefix(agentIndex, move, base.positions, nextPositions, chosenMoves)) continue;
            I_Location[] childNext = Arrays.copyOf(nextPositions, n);
            BitSet childMoved = (BitSet) movedUAs.clone();
            BitSet childVisited = (BitSet) visitedTargets.clone();
            Move[] childMoves = Arrays.copyOf(chosenMoves, n);
            childNext[agentIndex] = dst;
            childMoves[agentIndex] = move;
            boolean moved = !dst.equals(current);
            int childFuel = stepFuel + (moved ? 1 : 0);
            int childStepNewUA = stepNewUA;
            int uaIndex = context.uaIndexByAgentIndex[agentIndex];
            if (uaIndex >= 0 && moved && !childMoved.get(uaIndex)) {
                childMoved.set(uaIndex);
                childStepNewUA++;
            }
            if (dst.getCoordinate().equals(agent.target)) childVisited.set(agentIndex);
            MuaSearchState partial = MuaSearchState.partial(base, agentIndex + 1, childNext, childMoved,
                    childVisited, childMoves, 0, nextId(), childFuel, childStepNewUA);
            pushIfBetter(withHeuristic(partial, partialHeuristic(base, partial)));
        }
    }

    private int stepSST(MuaSearchState base) {
        int cost = 0;
        for (int agentIndex : context.assignedIndexes) {
            if (!base.visitedTargets.get(agentIndex)) cost++;
        }
        return cost;
    }

    private long partialHeuristic(MuaSearchState base, MuaSearchState partial) {
        MuaSearchState synthetic = MuaSearchState.full(base.time + 1,
                overlay(base.positions, partial.nextPositions, partial.nextAgent),
                partial.nextMovedUAs, partial.nextVisitedTargets, base.g, 0, base, null, partial.id);
        return heuristic.estimate(synthetic);
    }

    private I_Location[] overlay(I_Location[] base, I_Location[] next, int prefix) {
        I_Location[] copy = Arrays.copyOf(base, base.length);
        for (int i = 0; i < prefix; i++) copy[i] = next[i];
        return copy;
    }

    private List<I_Location> legalMoves(I_Location current) {
        List<I_Location> moves = new ArrayList<>(current.outgoingEdges());
        moves.sort(Comparator.comparing(l -> l.getCoordinate().toString()));
        if (current.getType() != Enum_MapLocationType.NO_STOP) moves.add(current);
        return moves;
    }

    private boolean conflictsWithPrefix(int agentIndex, Move move, I_Location[] basePositions,
                                        I_Location[] nextPositions, Move[] chosenMoves) {
        for (int j = 0; j < agentIndex; j++) {
            Move other = chosenMoves[j];
            if (other == null) continue;
            if (move.currLocation.equals(nextPositions[j])) return true;
            if (move.prevLocation.equals(other.currLocation) && move.currLocation.equals(other.prevLocation)) return true;
        }
        return false;
    }

    void pushIfBetter(MuaSearchState node) {
        MuaSearchState.MuaStateKey key = node.key();
        Long old = bestG.get(key);
        long g = effectiveG(node);
        if (old == null || g < old) {
            bestG.put(key, g);
            open.add(node);
            generatedNodes++;
        }
    }

    private Solution buildSolution(MuaSearchState goal) {
        List<MuaSearchState> states = new ArrayList<>();
        for (MuaSearchState n = goal; n != null; n = n.fullParent()) states.add(n);
        Collections.reverse(states);
        Solution solution = transientMAPFSettings.isTransientMAPF() ? new TransientMAPFSolution() : new Solution();
        for (Agent agent : context.agents) solution.putPlan(new SingleAgentPlan(agent));
        for (int i = 1; i < states.size(); i++) {
            Move[] moves = states.get(i).jointMovesFromParent;
            for (int a = 0; a < context.agents.size(); a++) {
                solution.getPlanFor(context.agents.get(a)).addMove(moves[a]);
            }
        }
        return solution;
    }

    private long nextId() {
        return serial++;
    }

    private final Comparator<MuaSearchState> nodeComparator = (a, b) -> {
        int byF = Long.compare(Math.addExact(effectiveG(a), a.h), Math.addExact(effectiveG(b), b.h));
        if (byF != 0) return byF;
        return Long.compare(a.id, b.id);
    };

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);
        if (instanceReport == null) return;
        instanceReport.putStringValue("MAPFUA_MA_objective", objective.reportName);
        instanceReport.putStringValue("MAPFUA_MA_heuristic", heuristic.name());
        instanceReport.putIntegerValue("MAPFUA_MA_main_generated", (int) generatedNodes);
        instanceReport.putIntegerValue("MAPFUA_MA_main_expanded", (int) expandedNodes);
        instanceReport.putIntegerValue("MAPFUA_MA_intermediate_OD_nodes", (int) intermediateNodes);
        instanceReport.putIntegerValue("MAPFUA_MA_full_joint_nodes", (int) fullNodes);
        MuaHeuristicMetrics hm = heuristic.metrics();
        instanceReport.putIntegerValue("MAPFUA_MA_heuristic_evaluations", (int) hm.evaluations);
        instanceReport.putIntegerValue("MAPFUA_MA_heuristic_generated", (int) hm.internalGenerated);
        instanceReport.putIntegerValue("MAPFUA_MA_heuristic_expanded", (int) hm.internalExpanded);
        instanceReport.putIntegerValue("MAPFUA_MA_heuristic_cache_hits", (int) hm.cacheHits);
        instanceReport.putIntegerValue("MAPFUA_MA_heuristic_time_ms", (int) (hm.nanos / 1_000_000L));
        instanceReport.putIntegerValue("MAPFUA_MA_root_h", (int) rootH);
        instanceReport.putIntegerValue("MAPFUA_MA_lazy_full", (int) hm.lazyFullEvaluations);
        instanceReport.putIntegerValue("MAPFUA_MA_lazy_inherited", (int) hm.lazyInheritedEvaluations);
        instanceReport.putIntegerValue("MAPFUA_MA_RDC_builds", (int) context.rdc.builds);
        instanceReport.putIntegerValue("MAPFUA_MA_RDC_queries", (int) context.rdc.queries);
        instanceReport.putIntegerValue("MAPFUA_MA_RDC_hits", (int) context.rdc.hits);
        instanceReport.putIntegerValue("MAPFUA_MA_RDC_misses", (int) context.rdc.misses);
        instanceReport.putIntegerValue("MAPFUA_MA_RDC_manhattan_fallbacks", (int) context.rdc.manhattanFallbacks);
        if (solution != null) {
            instanceReport.putIntegerValue("MAPFUA_MA_exact_MUA", solution.MUA());
            instanceReport.putIntegerValue("MAPFUA_MA_total_Fuel", solution.FUEL());
            instanceReport.putIntegerValue("MAPFUA_MA_assigned_Fuel", solution.Fuel_Assigned());
            instanceReport.putIntegerValue("MAPFUA_MA_unassigned_Fuel", solution.FUEL() - solution.Fuel_Assigned());
            instanceReport.putFloatValue(InstanceReport.StandardFields.solutionCost, objectiveValue(solution));
            instanceReport.putStringValue(InstanceReport.StandardFields.solutionCostFunction, objective.reportName);
        }
    }

    private int objectiveValue(Solution solution) {
        return switch (objective) {
            case MUA -> solution.MUA();
            case FUEL -> solution.FUEL();
            case SST -> solution.SST();
        };
    }

    @Override
    protected void releaseMemory() {
        super.releaseMemory();
        context = null;
        heuristic = null;
        open = null;
        bestG = null;
    }
}
