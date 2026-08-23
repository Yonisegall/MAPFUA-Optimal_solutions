package BasicMAPF.Solvers.AStart_OD;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import BasicMAPF.CostFunctions.I_SolutionCostFunction;
import BasicMAPF.CostFunctions.SumFuel;
import BasicMAPF.CostFunctions.SumMUA;
import BasicMAPF.CostFunctions.SumOfCosts;
import BasicMAPF.CostFunctions.SumServiceTimes;
import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.Coordinates.I_Coordinate;
import BasicMAPF.Instances.Maps.Enum_MapLocationType;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Instances.Maps.I_Map;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.CachingDistanceTableHeuristic;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.DistanceTableSingleAgentHeuristic;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.FuelGAndH;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.MUAGAndH;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.ServiceTimeGAndH;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.SingleAgentGAndH;
import BasicMAPF.Solvers.A_Solver;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;

/**
 * Coupled A* for MAPF / MAPFUA using Operator Decomposition (OD).
 *
 * Key idea:
 * - FullNode  : positions of all agents at time t
 * - PartialNode: a partially constructed joint move from time t to time t+1
 *
 * This solver does NOT use CBS constraints or low-level replanning.
 * It searches directly in the joint state-space.
 */
public class AStarOD_Solver extends A_Solver {

    private MAPF_Instance instance;
    private I_Map map;
    private List<Agent> agents;
    private int numAgents;
    private SingleAgentGAndH singleAgentGAndH;
    private final I_SolutionCostFunction costFunction;
    private PriorityQueue<ODNode> open;
    private Map<StateKey, Integer> bestG;
    private long serialCounter;
    private boolean enforceStaticUA = false;
    private boolean[] isStaticUAByIndex = null;
    private java.util.HashSet<I_Location> staticBlockedLocations = new java.util.HashSet<>();
    private Set<Agent> staticUA = null;
    private final boolean staticObstaclesForUnassignedAgents;

    public AStarOD_Solver(@Nullable SingleAgentGAndH singleAgentGAndH,
                          @Nullable I_SolutionCostFunction costFunction,
                          @Nullable TransientMAPFSettings transientMAPFSettings,
                           boolean staticObstaclesForUnassignedAgents) {

        this.singleAgentGAndH = singleAgentGAndH;
        this.costFunction = Objects.requireNonNullElseGet(costFunction, SumOfCosts::new);
        this.transientMAPFSettings = Objects.requireNonNullElse(transientMAPFSettings, TransientMAPFSettings.defaultRegularMAPF);
        this.staticObstaclesForUnassignedAgents = staticObstaclesForUnassignedAgents;

        // Naming like CBS
        if (this.transientMAPFSettings.isTransientMAPF()) {
            super.name = "AStarOD_Dynamic_UA";
        } else if (this.staticObstaclesForUnassignedAgents) {
            super.name = "AStarOD_Static_UA";
        } else {
            super.name = "AStarOD";
        }
    }

    @Override
    protected void init(MAPF_Instance instance, RunParameters runParameters) {
        super.init(instance, runParameters);
        this.instance = instance;
        this.map = instance.map;
        this.agents = new ArrayList<>(instance.agents);
        this.numAgents = this.agents.size();

        // ═══════════════ Static UA handling (via legalMoves, NOT via constraints) ═══════════════
        // OD doesn't use constraints. Static UA are handled by restricting their moves to "wait only"
        // and blocking other agents from entering their cells.

        this.enforceStaticUA = false;
        this.isStaticUAByIndex = null;
        this.staticBlockedLocations.clear();

        // Case 1: ALL UA are static obstacles (like CBS_Static_UA)
        if (this.staticObstaclesForUnassignedAgents && (this.staticUA == null || this.staticUA.isEmpty())) {
            java.util.HashSet<Integer> allUAIds = new java.util.HashSet<>();
            for (Agent a : this.agents) {
                if (a.isUnassigned) allUAIds.add(a.iD);
            }
            if (!allUAIds.isEmpty()) {
                setupStaticUAByIds(allUAIds);
            }
        }

        // Case 2: SOME UA are static obstacles (set by MinMUAOD via setStaticUA)
        if (this.staticUA != null && !this.staticUA.isEmpty()) {
            java.util.HashSet<Integer> ids = new java.util.HashSet<>();
            for (Agent a : this.staticUA) ids.add(a.iD);
            setupStaticUAByIds(ids);
        }

        // ═══════════════ Heuristic setup (same as CBS) ═══════════════

        if (this.singleAgentGAndH == null) {
            this.singleAgentGAndH = new DistanceTableSingleAgentHeuristic(this.agents, this.map);
        }
        if (this.singleAgentGAndH instanceof CachingDistanceTableHeuristic caching) {
            caching.setCurrentMap(this.map);
        }

        // Wrap heuristic based on cost function (mirrors CBS logic)
        if (this.costFunction instanceof SumServiceTimes && !(this.singleAgentGAndH instanceof ServiceTimeGAndH)) {
            this.singleAgentGAndH = new ServiceTimeGAndH(this.singleAgentGAndH);
        } else if (this.costFunction instanceof SumFuel && !(this.singleAgentGAndH instanceof FuelGAndH)) {
            this.singleAgentGAndH = new FuelGAndH(this.singleAgentGAndH);
        } else if (this.costFunction instanceof SumMUA && !(this.singleAgentGAndH instanceof MUAGAndH)) {
            this.singleAgentGAndH = new MUAGAndH(this.singleAgentGAndH);
        }

        // ═══════════════ Search structures ═══════════════

        this.open = new PriorityQueue<>(odComparator);
        this.bestG = new HashMap<>();
        this.generatedNodes = 0;
        this.expandedNodes = 0;
        this.serialCounter = 0;
    }

    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        return solveOD();
    }

    private Solution solveOD() {
        FullNode root = createRoot();

        if (isGoal(root)) {
            return buildSolution(root);
        }

        pushIfBetter(root);

        while (!open.isEmpty()) {
            if (checkTimeout()) {
                return null;
            }

            ODNode current = open.poll();

            Integer knownBest = bestG.get(current.key());
            if (knownBest == null || current.g != knownBest) {
                continue; // stale queue entry
            }

            if (current instanceof FullNode full && isGoal(full)) {
                return buildSolution(full);
            }

            this.expandedNodes++;

            if (current instanceof FullNode full) {
                expandFull(full);
            } else {
                expandPartial((PartialNode) current);
            }
        }

        return null;
    }

    public void setStaticUA(Set<Agent> staticUA) {
        this.staticUA = staticUA;
    }

    public void setStaticUAById(java.util.Set<Integer> staticUAIds) {
        if (staticUAIds == null || staticUAIds.isEmpty()) {
            enforceStaticUA = false;
            isStaticUAByIndex = null;
            staticBlockedLocations.clear();
            return;
        }

        enforceStaticUA = true;
        isStaticUAByIndex = new boolean[numAgents];
        staticBlockedLocations.clear();

        for (int i = 0; i < numAgents; i++) {
            Agent a = agents.get(i);
            if (staticUAIds.contains(a.iD)) {
                isStaticUAByIndex[i] = true;
                staticBlockedLocations.add(map.getMapLocation(a.source));
            }
        }
    }

    private void setupStaticUAByIds(java.util.Set<Integer> ids) {
        if (ids == null || ids.isEmpty()) return;

        this.enforceStaticUA = true;
        this.isStaticUAByIndex = new boolean[numAgents];
        this.staticBlockedLocations.clear();

        for (int i = 0; i < numAgents; i++) {
            Agent a = agents.get(i);
            if (ids.contains(a.iD)) {
                isStaticUAByIndex[i] = true;
                staticBlockedLocations.add(map.getMapLocation(a.source));
            }
        }
    }

    private FullNode createRoot() {
        I_Location[] startPositions = new I_Location[numAgents];
        boolean[] visitedTargets = new boolean[numAgents];

        for (int i = 0; i < numAgents; i++) {
            Agent a = agents.get(i);
            I_Location source = map.getMapLocation(a.source);
            startPositions[i] = source;

            // MAPFUA trick:
            // Unassigned agents have source == target, so they are considered to have already visited the target at t=0.
            visitedTargets[i] = a.isUnassigned;
        }

        float h = heuristicForFull(startPositions, visitedTargets);

        return new FullNode(0, startPositions, visitedTargets, 0, h, null, null, nextID());
    }

    /**
     * Goal condition:
     * all agents have visited their target at least once.
     *
     * For UA agents with source==target, this is already true at time 0.
     */
    private boolean isGoal(FullNode node) {
        if (this.transientMAPFSettings != null && this.transientMAPFSettings.isTransientMAPF()) {
            // Transient: Only need to have visited target at some point, can end anywhere. Check visitedTargets array.
            for (boolean visited : node.visitedTargets) {
                if (!visited) return false;
            }
            return true;
        } else {
            // Regular MAPF: All agents must be at their target location at the current time.
            for (int i = 0; i < numAgents; i++) {
                Agent a = agents.get(i);
                if (!node.positions[i].getCoordinate().equals(a.target)) return false;
            }
            return true;
        }
    }

    private void expandFull(@NotNull FullNode node) {

        // Start constructing the joint move for time t -> t+1 from agent 0.
        expandPartialFrom(node, 0, new I_Location[numAgents], new boolean[numAgents], new Move[numAgents], 0);
    }

    /**
     * Creates the first layer of partial nodes from a FullNode.
     */
    private void expandPartialFrom(@NotNull FullNode baseNode,
                                    int nextAgent,
                                    I_Location[] nextPositions,
                                    boolean[] nextVisitedTargets,
                                    Move[] chosenMoves,
                                    int accumulatedStepCost) {

        if (nextAgent == numAgents) {
            
            // Joint move completed -> produce next FullNode.
            I_Location[] fullNextPositions = Arrays.copyOf(nextPositions, numAgents);
            boolean[] fullNextVisited = Arrays.copyOf(nextVisitedTargets, numAgents);
            Move[] fullJointMoves = Arrays.copyOf(chosenMoves, numAgents);

            int newG = baseNode.g + accumulatedStepCost;
            float newH = heuristicForFull(fullNextPositions, fullNextVisited);

            FullNode child = new FullNode(
                    baseNode.time + 1,
                    fullNextPositions,
                    fullNextVisited,
                    newG,
                    newH,
                    baseNode,
                    fullJointMoves,
                    nextID()
            );
            pushIfBetter(child);
            return;
        }

        Agent agent = agents.get(nextAgent);
        I_Location currLocation = baseNode.positions[nextAgent];

        for (I_Location dst : legalMoves(currLocation, nextAgent)) {
            Move move = new Move(agent, baseNode.time + 1, currLocation, dst);

            if (conflictsWithAlreadyChosen(nextAgent, move, baseNode.positions, nextPositions, chosenMoves)) {
                continue;
            }

            I_Location[] childNextPositions = Arrays.copyOf(nextPositions, numAgents);
            boolean[] childNextVisited = Arrays.copyOf(nextVisitedTargets, numAgents);
            Move[] childChosenMoves = Arrays.copyOf(chosenMoves, numAgents);

            childNextPositions[nextAgent] = dst;
            childNextVisited[nextAgent] = baseNode.visitedTargets[nextAgent] ||
                    dst.getCoordinate().equals(agent.target);
            childChosenMoves[nextAgent] = move;

            int childAccumulatedStepCost =
                    accumulatedStepCost + stepCostForAgent(baseNode.visitedTargets[nextAgent], move);

            float h = heuristicForPartial(baseNode, childNextPositions, childNextVisited, nextAgent + 1);
            PartialNode child = new PartialNode(
                    baseNode.time,
                    baseNode.positions,
                    baseNode.visitedTargets,
                    childNextPositions,
                    childNextVisited,
                    childChosenMoves,
                    nextAgent + 1,
                    baseNode.g,
                    h,
                    baseNode,
                    nextID(),
                    childAccumulatedStepCost
            );
            pushIfBetter(child);
        }
    }

    private void expandPartial(@NotNull PartialNode node) {
        if (node.nextAgent == numAgents) {
            I_Location[] fullNextPositions = Arrays.copyOf(node.nextPositions, numAgents);
            boolean[] fullNextVisited = Arrays.copyOf(node.nextVisitedTargets, numAgents);
            Move[] fullJointMoves = Arrays.copyOf(node.chosenMoves, numAgents);

            int newG = node.g + node.accumulatedStepCost;
            float newH = heuristicForFull(fullNextPositions, fullNextVisited);

            FullNode child = new FullNode(
                    node.time + 1,
                    fullNextPositions,
                    fullNextVisited,
                    newG,
                    newH,
                    node.baseFullParent,
                    fullJointMoves,
                    nextID()
            );
            pushIfBetter(child);
            return;
        }

        int i = node.nextAgent;
        Agent agent = agents.get(i);
        I_Location currLocation = node.basePositions[i];
        
        for (I_Location dst : legalMoves(currLocation, i)) {
            Move move = new Move(agent, node.time + 1, currLocation, dst);

            if (conflictsWithAlreadyChosen(i, move, node.basePositions, node.nextPositions, node.chosenMoves)) {
                continue;
            }

            I_Location[] childNextPositions = Arrays.copyOf(node.nextPositions, numAgents);
            boolean[] childNextVisited = Arrays.copyOf(node.nextVisitedTargets, numAgents);
            Move[] childChosenMoves = Arrays.copyOf(node.chosenMoves, numAgents);

            childNextPositions[i] = dst;
            childNextVisited[i] = node.baseVisitedTargets[i] ||
                    dst.getCoordinate().equals(agent.target);
            childChosenMoves[i] = move;

            int childAccumulatedStepCost =
                    node.accumulatedStepCost + stepCostForAgent(node.baseVisitedTargets[i], move);

            float h = heuristicForPartial(
                    node.basePositions,
                    node.baseVisitedTargets,
                    childNextPositions,
                    childNextVisited,
                    i + 1
            );

            PartialNode child = new PartialNode(
                    node.time,
                    node.basePositions,
                    node.baseVisitedTargets,
                    childNextPositions,
                    childNextVisited,
                    childChosenMoves,
                    i + 1,
                    node.g,
                    h,
                    node.baseFullParent,
                    nextID(),
                    childAccumulatedStepCost
            );
            pushIfBetter(child);
        }
    }

    /**
     * Vertex + edge conflict check against agents 0..agentIndex-1 that were already assigned in this partial node.
     */
    private boolean conflictsWithAlreadyChosen(int agentIndex,
                                               Move candidateMove,
                                               I_Location[] basePositions,
                                               I_Location[] nextPositions,
                                               Move[] chosenMoves) {
        for (int j = 0; j < agentIndex; j++) {
            Move other = chosenMoves[j];
            if (other == null) {
                continue;
            }

            // Vertex conflict: same destination at time t+1
            if (candidateMove.currLocation.equals(nextPositions[j])) {
                return true;
            }

            // Edge conflict: swap on the same edge
            if (candidateMove.prevLocation.equals(other.currLocation) &&
                candidateMove.currLocation.equals(other.prevLocation)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Returns outgoing neighbors plus wait, unless NO_STOP.
     */
    private List<I_Location> legalMoves(@NotNull I_Location location, int agentIndex) {
        // אם זה סטטי – אסור לזוז
        if (enforceStaticUA && isStaticUAByIndex != null && isStaticUAByIndex[agentIndex]) {
            if (location.getType() != Enum_MapLocationType.NO_STOP) {
                return java.util.List.of(location);
            }
            return java.util.List.of();
        }

        List<I_Location> result = new ArrayList<>(location.outgoingEdges());
        if (location.getType() != Enum_MapLocationType.NO_STOP) {
            result.add(location);
        }

        // אסור להיכנס לתאים של סטטיים
        if (enforceStaticUA && !staticBlockedLocations.isEmpty()) {
            result.removeIf(staticBlockedLocations::contains);
        }
        return result;
    }

    /**
     * Reuses your low-level cost semantics.
     *
     * baseVisitedTarget == true means:
     * "this agent has already visited target before this move"
     *
     * That matches the meaning of the second parameter in your wrappers.
     */
    private int stepCostForAgent(boolean baseVisitedTarget, Move move) {
        return singleAgentGAndH.cost(move, baseVisitedTarget);
    }

    private float heuristicForFull(I_Location[] positions, boolean[] visitedTargets) {
        float h = 0;
        for (int i = 0; i < numAgents; i++) {
            if (!visitedTargets[i]) {
                h += singleAgentHeuristic(i, positions[i]);
            }
        }
        return h;
    }

    private float heuristicForPartial(FullNode baseNode,
                                      I_Location[] partialNextPositions,
                                      boolean[] partialNextVisitedTargets,
                                      int prefixLength) {
        return heuristicForPartial(
                baseNode.positions,
                baseNode.visitedTargets,
                partialNextPositions,
                partialNextVisitedTargets,
                prefixLength
        );
    }

    /**
     * For agents already assigned in the partial node, evaluate from next position.
     * For the rest, evaluate from current/base position.
     */
    private float heuristicForPartial(I_Location[] basePositions,
                                      boolean[] baseVisitedTargets,
                                      I_Location[] partialNextPositions,
                                      boolean[] partialNextVisitedTargets,
                                      int prefixLength) {
        float h = 0;
        for (int i = 0; i < numAgents; i++) {
            boolean visited;
            I_Location pos;

            if (i < prefixLength && partialNextPositions[i] != null) {
                visited = partialNextVisitedTargets[i];
                pos = partialNextPositions[i];
            } else {
                visited = baseVisitedTargets[i];
                pos = basePositions[i];
            }

            if (!visited) {
                h += singleAgentHeuristic(i, pos);
            }
        }
        return h;
    }

    private float singleAgentHeuristic(int agentIndex, I_Location loc) {
        Agent a = agents.get(agentIndex);
        return singleAgentGAndH.getHToTargetFromLocation(a.target, loc);
    }

    private void pushIfBetter(@NotNull ODNode node) {
        StateKey key = node.key();
        Integer oldG = bestG.get(key);
        if (oldG == null || node.g < oldG) {
            bestG.put(key, node.g);
            open.add(node);
            this.generatedNodes++;
        }
    }

    private long nextID() {
        return serialCounter++;
    }

    private Solution buildSolution(@NotNull FullNode goal) {
        List<FullNode> fullPath = new ArrayList<>();
        FullNode current = goal;
        while (current != null) {
            fullPath.add(current);
            current = current.parentFull();
        }
        Collections.reverse(fullPath);

        Solution solution = new Solution();
        for (Agent agent : agents) {
            solution.putPlan(new SingleAgentPlan(agent));
        }

        // fullPath[0] is root at t=0 with no joint move.
        for (int idx = 1; idx < fullPath.size(); idx++) {
            FullNode stepNode = fullPath.get(idx);
            for (int i = 0; i < numAgents; i++) {
                solution.getPlanFor(agents.get(i)).addMove(stepNode.jointMoves[i]);
            }
        }

        return solution;
    }

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);
        if (instanceReport != null) {
            // instanceReport.putIntegerValue(InstanceReport.StandardFields.generatedNodesLowLevel, (int) generatedNodes);
            // instanceReport.putIntegerValue(InstanceReport.StandardFields.expandedNodesLowLevel, (int) expandedNodes);
            if (solution != null) {
                instanceReport.putFloatValue(InstanceReport.StandardFields.solutionCost, costFunction.solutionCost(solution));
                instanceReport.putStringValue(InstanceReport.StandardFields.solutionCostFunction, costFunction.name());
            }
        }
    }

    @Override
    protected void releaseMemory() {
        super.releaseMemory();
        this.instance = null;
        this.map = null;
        this.agents = null;
        this.singleAgentGAndH = null;
        this.open = null;
        this.bestG = null;
    }

    private final Comparator<ODNode> odComparator = (a, b) -> {
        int byF = Float.compare(a.f(), b.f());
        if (byF != 0) return byF;

        int byH = Float.compare(a.h, b.h);
        if (byH != 0) return byH;

        int byG = Integer.compare(b.g, a.g); // higher g first, similar to your single-agent tie-break
        if (byG != 0) return byG;

        return Long.compare(a.serialID, b.serialID);
    };

    // =====================================================================================
    // Node hierarchy
    // =====================================================================================

    private abstract static class ODNode {
        final int time;
        final int g;
        final float h;
        final long serialID;

        ODNode(int time, int g, float h, long serialID) {
            this.time = time;
            this.g = g;
            this.h = h;
            this.serialID = serialID;
        }

        float f() {
            return g + h;
        }

        abstract StateKey key();
    }

    /**
     * Real world state at time t.
     */
    private static class FullNode extends ODNode {
        final I_Location[] positions;
        final boolean[] visitedTargets;
        final FullNode parent;
        final Move[] jointMoves; // moves used to get here from parent

        FullNode(int time,
                 I_Location[] positions,
                 boolean[] visitedTargets,
                 int g,
                 float h,
                 FullNode parent,
                 Move[] jointMoves,
                 long serialID) {
            super(time, g, h, serialID);
            this.positions = positions;
            this.visitedTargets = visitedTargets;
            this.parent = parent;
            this.jointMoves = jointMoves;
        }

        FullNode parentFull() {
            return parent;
        }

        @Override
        StateKey key() {
            return StateKey.full(time, positions, visitedTargets);
        }
    }

    /**
     * Intermediate OD node:
     * we are still assigning the joint move for time t -> t+1.
     */
    private static class PartialNode extends ODNode {
        final I_Location[] basePositions;
        final boolean[] baseVisitedTargets;

        final I_Location[] nextPositions;
        final boolean[] nextVisitedTargets;
        final Move[] chosenMoves;

        final int nextAgent;
        final FullNode baseFullParent;

        /**
         * Cost accumulated inside the partially built timestep.
         * It is NOT yet added to g; it is added only when a FullNode is created.
         */
        final int accumulatedStepCost;

        PartialNode(int time,
                    I_Location[] basePositions,
                    boolean[] baseVisitedTargets,
                    I_Location[] nextPositions,
                    boolean[] nextVisitedTargets,
                    Move[] chosenMoves,
                    int nextAgent,
                    int g,
                    float h,
                    FullNode baseFullParent,
                    long serialID,
                    int accumulatedStepCost) {
            super(time, g, h, serialID);
            this.basePositions = basePositions;
            this.baseVisitedTargets = baseVisitedTargets;
            this.nextPositions = nextPositions;
            this.nextVisitedTargets = nextVisitedTargets;
            this.chosenMoves = chosenMoves;
            this.nextAgent = nextAgent;
            this.baseFullParent = baseFullParent;
            this.accumulatedStepCost = accumulatedStepCost;
        }

        @Override
        StateKey key() {
            return StateKey.partial(
                    time,
                    basePositions,
                    baseVisitedTargets,
                    nextPositions,
                    nextVisitedTargets,
                    nextAgent
            );
        }
    }

    // =====================================================================================
    // State-key for duplicate detection
    // =====================================================================================

    private static class StateKey {
        private final boolean partial;
        private final int time;
        private final int nextAgent;

        private final I_Coordinate[] baseCoords;
        private final boolean[] baseVisited;

        private final I_Coordinate[] nextCoords;
        private final boolean[] nextVisited;

        private StateKey(boolean partial,
                         int time,
                         int nextAgent,
                         I_Coordinate[] baseCoords,
                         boolean[] baseVisited,
                         I_Coordinate[] nextCoords,
                         boolean[] nextVisited) {
            this.partial = partial;
            this.time = time;
            this.nextAgent = nextAgent;
            this.baseCoords = baseCoords;
            this.baseVisited = baseVisited;
            this.nextCoords = nextCoords;
            this.nextVisited = nextVisited;
        }

        static StateKey full(int time, I_Location[] positions, boolean[] visitedTargets) {
            return new StateKey(
                    false,
                    time,
                    -1,
                    coordsOf(positions),
                    Arrays.copyOf(visitedTargets, visitedTargets.length),
                    null,
                    null
            );
        }

        static StateKey partial(int time,
                                I_Location[] basePositions,
                                boolean[] baseVisitedTargets,
                                I_Location[] nextPositions,
                                boolean[] nextVisitedTargets,
                                int nextAgent) {
            I_Coordinate[] nextCoords = new I_Coordinate[nextAgent];
            boolean[] nextVisited = new boolean[nextAgent];
            for (int i = 0; i < nextAgent; i++) {
                nextCoords[i] = nextPositions[i].getCoordinate();
                nextVisited[i] = nextVisitedTargets[i];
            }

            return new StateKey(
                    true,
                    time,
                    nextAgent,
                    coordsOf(basePositions),
                    Arrays.copyOf(baseVisitedTargets, baseVisitedTargets.length),
                    nextCoords,
                    nextVisited
            );
        }

        private static I_Coordinate[] coordsOf(I_Location[] locs) {
            I_Coordinate[] arr = new I_Coordinate[locs.length];
            for (int i = 0; i < locs.length; i++) {
                arr[i] = locs[i].getCoordinate();
            }
            return arr;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof StateKey that)) return false;
            return partial == that.partial
                    && time == that.time
                    && nextAgent == that.nextAgent
                    && Arrays.equals(baseCoords, that.baseCoords)
                    && Arrays.equals(baseVisited, that.baseVisited)
                    && Arrays.equals(nextCoords, that.nextCoords)
                    && Arrays.equals(nextVisited, that.nextVisited);
        }

        @Override
        public int hashCode() {
            int result = Objects.hash(partial, time, nextAgent);
            result = 31 * result + Arrays.hashCode(baseCoords);
            result = 31 * result + Arrays.hashCode(baseVisited);
            result = 31 * result + Arrays.hashCode(nextCoords);
            result = 31 * result + Arrays.hashCode(nextVisited);
            return result;
        }
    }
}
