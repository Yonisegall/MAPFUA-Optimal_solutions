package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.DataTypesAndStructures.Timeout;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.Coordinates.I_Coordinate;
import BasicMAPF.Instances.Maps.Enum_MapLocationType;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.AStart_OD.MAPFUA_MA.MapfuaObjective;
import BasicMAPF.Solvers.ConstraintsAndConflicts.A_Conflict;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.GoalConstraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.I_ConstraintSet;
import TransientMAPF.TransientMAPFSettings;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * CBS* low-level search for one meta-agent under MAPFUA.
 *
 * The paper's two-phase low level is adapted as follows: while constraints applicable to this meta-agent remain in the
 * future, duplicate keys include absolute time. Once no such constraint remains, absolute time is dropped, but the key
 * still contains the full joint configuration, transient target-visit history, and the moved-UA mask. This is safe
 * because future pure MUA/Fuel/SST transition costs depend only on the stored history and current positions, not on
 * elapsed absolute time once all explicit future constraints are gone. UA agents are never forced back to their source.
 */
public class CBSStarJointLowLevel {
    private final MAPF_Instance instance;
    private final TransientMAPFSettings transientSettings;
    private final Timeout timeout;
    private final MapfuaObjective objective;
    private int generated;
    private int expanded;

    public CBSStarJointLowLevel(MAPF_Instance instance, TransientMAPFSettings transientSettings, Timeout timeout) {
        this(instance, transientSettings, timeout, MapfuaObjective.MUA);
    }

    public CBSStarJointLowLevel(MAPF_Instance instance, TransientMAPFSettings transientSettings, Timeout timeout,
                                MapfuaObjective objective) {
        this.instance = instance;
        this.transientSettings = transientSettings;
        this.timeout = timeout;
        this.objective = objective;
    }

    public int generated() {
        return generated;
    }

    public int expanded() {
        return expanded;
    }

    public Solution solve(MetaAgent metaAgent, I_ConstraintSet constraints) {
        List<Agent> agents = metaAgent.agents();
        int n = agents.size();
        I_Location[] start = new I_Location[n];
        boolean[] visited = new boolean[n];
        boolean[] movedUAs = new boolean[n];
        for (int i = 0; i < n; i++) {
            Agent agent = agents.get(i);
            start[i] = instance.map.getMapLocation(agent.source);
            visited[i] = agent.isUnassigned || start[i].getCoordinate().equals(agent.target);
        }

        PriorityQueue<Node> open = new PriorityQueue<>(Comparator
                .comparingInt((Node node) -> node.g)
                .thenComparingLong(node -> node.id));
        Map<StateKey, Integer> bestG = new HashMap<>();
        generated = 0;
        expanded = 0;
        Node root = new Node(agents, start, visited, movedUAs, 0, 0, null, null, generated++);
        putIfBetter(root, hasFutureConstraint(agents, root.time, constraints), bestG, open);

        while (!open.isEmpty()) {
            if (timeout.isTimeoutExceeded()) return null;
            Node current = open.poll();
            boolean timed = hasFutureConstraint(agents, current.time, constraints);
            Integer known = bestG.get(StateKey.of(current, timed));
            if (known == null || known != current.g) continue;
            expanded++;

            if (isGoal(current) && acceptsForever(current, constraints)) {
                return reconstruct(metaAgent, current);
            }

            expand(current, timed, constraints, bestG, open);
        }
        return null;
    }

    private void expand(Node current, boolean timed, I_ConstraintSet constraints,
                        Map<StateKey, Integer> bestG, PriorityQueue<Node> open) {
        ArrayList<I_Location[]> successors = new ArrayList<>();
        buildJointMoves(current, constraints, 0, new I_Location[current.positions.length],
                new Move[current.positions.length], successors);
        for (I_Location[] next : successors) {
            boolean[] nextVisited = Arrays.copyOf(current.visitedTargets, current.visitedTargets.length);
            boolean[] nextMoved = Arrays.copyOf(current.movedUAs, current.movedUAs.length);
            int newMovedUAs = 0;
            int fuel = 0;
            Move[] moves = new Move[next.length];
            int sst = unservedAssigned(current);
            for (int i = 0; i < next.length; i++) {
                Agent agent = current.metaAgents.get(i);
                moves[i] = new Move(agent, current.time + 1, current.positions[i], next[i]);
                if (!current.positions[i].equals(next[i])) {
                    fuel++;
                }
                if (next[i].getCoordinate().equals(agent.target)) {
                    nextVisited[i] = true;
                }
                if (agent.isUnassigned && !nextMoved[i] && !current.positions[i].equals(next[i])) {
                    nextMoved[i] = true;
                    newMovedUAs++;
                }
            }
            Node child = new Node(current.metaAgents, next, nextVisited, nextMoved, current.time + 1,
                    current.g + (int) objective.stepCost(newMovedUAs, fuel, sst), current, moves, generated++);
            putIfBetter(child, hasFutureConstraint(current.metaAgents, child.time, constraints), bestG, open);
        }
    }

    private void buildJointMoves(Node current, I_ConstraintSet constraints, int index, I_Location[] next,
                                 Move[] moves, List<I_Location[]> successors) {
        if (timeout.isTimeoutExceeded()) return;
        if (index == current.positions.length) {
            successors.add(Arrays.copyOf(next, next.length));
            return;
        }

        Agent agent = current.metaAgents.get(index);
        I_Location from = current.positions[index];
        ArrayList<I_Location> candidates = new ArrayList<>(from.outgoingEdges());
        if (from.getType() != Enum_MapLocationType.NO_STOP) {
            candidates.add(from);
        }
        candidates.sort(Comparator.comparing(loc -> loc.getCoordinate().toString()));

        for (I_Location to : candidates) {
            Move move = new Move(agent, current.time + 1, from, to);
            if (constraints != null && constraints.rejects(move)) continue;
            if (conflictsWithPrefix(move, moves, index)) continue;
            next[index] = to;
            moves[index] = move;
            buildJointMoves(current, constraints, index + 1, next, moves, successors);
            next[index] = null;
            moves[index] = null;
        }
    }

    private boolean conflictsWithPrefix(Move move, Move[] chosen, int index) {
        for (int i = 0; i < index; i++) {
            if (A_Conflict.haveConflicts(move, chosen[i])) {
                return true;
            }
        }
        return false;
    }

    private void putIfBetter(Node node, boolean timed, Map<StateKey, Integer> bestG, PriorityQueue<Node> open) {
        StateKey key = StateKey.of(node, timed);
        Integer old = bestG.get(key);
        if (old == null || node.g < old) {
            bestG.put(key, node.g);
            open.add(node);
        }
    }

    private boolean isGoal(Node node) {
        for (int i = 0; i < node.metaAgents.size(); i++) {
            Agent agent = node.metaAgents.get(i);
            if (agent.isUnassigned) continue;
            if (transientSettings != null && transientSettings.isTransientMAPF()) {
                if (!node.visitedTargets[i]) return false;
            } else if (!node.positions[i].getCoordinate().equals(agent.target)) {
                return false;
            }
        }
        return true;
    }

    private int unservedAssigned(Node node) {
        int sst = 0;
        for (int i = 0; i < node.metaAgents.size(); i++) {
            Agent agent = node.metaAgents.get(i);
            if (!agent.isUnassigned && !node.visitedTargets[i]) sst++;
        }
        return sst;
    }

    private boolean acceptsForever(Node node, I_ConstraintSet constraints) {
        if (constraints == null) return true;
        for (int i = 0; i < node.metaAgents.size(); i++) {
            Move stay = new Move(node.metaAgents.get(i), Math.max(1, node.time), node.positions[i], node.positions[i]);
            if (!constraints.acceptsForever(stay)) return false;
        }
        return true;
    }

    private boolean hasFutureConstraint(List<Agent> agents, int time, I_ConstraintSet constraints) {
        if (constraints == null) return false;
        for (ArrayList<Constraint> atLocation : constraints.getLocationConstraintsTimeSorted().values()) {
            for (Constraint constraint : atLocation) {
                if (appliesToAny(constraint, agents) &&
                        ((constraint.prevLocation == null && constraint.time > time) ||
                                (constraint.prevLocation != null && constraint.time >= time))) {
                    return true;
                }
            }
        }
        for (GoalConstraint goal : constraints.getGoalConstraints().values()) {
            if (appliesToAny(goal, agents) && goal.time > time) {
                return true;
            }
        }
        return false;
    }

    private boolean appliesToAny(Constraint constraint, List<Agent> agents) {
        return constraint.agent == null || agents.contains(constraint.agent);
    }

    private Solution reconstruct(MetaAgent metaAgent, Node goal) {
        ArrayList<Node> states = new ArrayList<>();
        for (Node node = goal; node != null; node = node.parent) {
            states.add(node);
        }
        Collections.reverse(states);

        List<Agent> agents = metaAgent.agents();
        ArrayList<ArrayList<Move>> byAgent = new ArrayList<>();
        for (int i = 0; i < agents.size(); i++) byAgent.add(new ArrayList<>());
        for (int i = 1; i < states.size(); i++) {
            Move[] moves = states.get(i).movesFromParent;
            for (int a = 0; a < agents.size(); a++) {
                byAgent.get(a).add(moves[a]);
            }
        }
        for (int i = 0; i < agents.size(); i++) {
            if (byAgent.get(i).isEmpty()) {
                I_Location source = instance.map.getMapLocation(agents.get(i).source);
                byAgent.get(i).add(new Move(agents.get(i), 1, source, source));
            }
        }
        Solution solution = new Solution();
        for (int i = 0; i < agents.size(); i++) {
            solution.putPlan(new SingleAgentPlan(agents.get(i), byAgent.get(i)));
        }
        return solution;
    }

    private static final class Node {
        final List<Agent> metaAgents;
        final I_Location[] positions;
        final boolean[] visitedTargets;
        final boolean[] movedUAs;
        final int time;
        final int g;
        final Node parent;
        final Move[] movesFromParent;
        final long id;

        Node(List<Agent> metaAgents, I_Location[] positions, boolean[] visitedTargets, boolean[] movedUAs, int time, int g,
             Node parent, Move[] movesFromParent, long id) {
            this.metaAgents = metaAgents;
            this.positions = positions;
            this.visitedTargets = visitedTargets;
            this.movedUAs = movedUAs;
            this.time = time;
            this.g = g;
            this.parent = parent;
            this.movesFromParent = movesFromParent;
            this.id = id;
        }
    }

    private static final class StateKey {
        final boolean timed;
        final int time;
        final I_Coordinate[] coords;
        final boolean[] visitedTargets;
        final boolean[] movedUAs;

        private StateKey(boolean timed, int time, I_Coordinate[] coords, boolean[] visitedTargets, boolean[] movedUAs) {
            this.timed = timed;
            this.time = timed ? time : -1;
            this.coords = coords;
            this.visitedTargets = visitedTargets;
            this.movedUAs = movedUAs;
        }

        static StateKey of(Node node, boolean timed) {
            I_Coordinate[] coords = new I_Coordinate[node.positions.length];
            for (int i = 0; i < coords.length; i++) {
                coords[i] = node.positions[i].getCoordinate();
            }
            return new StateKey(timed, node.time, coords,
                    Arrays.copyOf(node.visitedTargets, node.visitedTargets.length),
                    Arrays.copyOf(node.movedUAs, node.movedUAs.length));
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof StateKey key)) return false;
            return timed == key.timed && time == key.time && Arrays.equals(coords, key.coords)
                    && Arrays.equals(visitedTargets, key.visitedTargets) && Arrays.equals(movedUAs, key.movedUAs);
        }

        @Override
        public int hashCode() {
            int result = Boolean.hashCode(timed);
            result = 31 * result + time;
            result = 31 * result + Arrays.hashCode(coords);
            result = 31 * result + Arrays.hashCode(visitedTargets);
            result = 31 * result + Arrays.hashCode(movedUAs);
            return result;
        }
    }
}
