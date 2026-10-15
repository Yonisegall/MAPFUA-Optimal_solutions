package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.Instances.Maps.I_Location;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * MAPFUA capture heuristics (ZERO, MAX, IH, MH). Only currently-unmoved original UAs are capturable
 * obstacles; already-moved UAs were paid for in g and are relaxed away. Only active AAs (target not yet
 * visited under reach-goal-once) are considered.
 */
final class MapfuaCaptureHeuristic implements MuaHeuristic {
    private final MuaProblemContext context;
    private final MapfuaObjective objective;
    private final MuaHeuristicType type;
    private final boolean lazyEvaluation;
    private final MuaHeuristicMetrics metrics = new MuaHeuristicMetrics();

    MapfuaCaptureHeuristic(MuaProblemContext context, MapfuaObjective objective, MuaHeuristicType type, boolean lazyEvaluation) {
        this.context = context;
        this.objective = objective;
        this.type = type;
        this.lazyEvaluation = lazyEvaluation;
    }

    @Override
    public long estimate(MuaSearchState state) {
        metrics.evaluations++;
        if (type == MuaHeuristicType.ZERO) return 0;
        if (lazyEvaluation && canInheritCaptureLowerBound(state)) {
            metrics.lazyInheritedEvaluations++;
            return state.parent.h;
        }
        metrics.lazyFullEvaluations++;
        return switch (type) {
            case MAX -> max(state);
            case INCREASING -> increasing(state);
            case MERGE -> merge(state);
            case ZERO -> 0;
        };
    }

    private boolean canInheritCaptureLowerBound(MuaSearchState state) {
        if (state == null || state.parent == null) return false;
        if (!state.movedUAs.equals(state.parent.movedUAs)) return false;
        if (!state.visitedTargets.equals(state.parent.visitedTargets)) return false;
        for (int agentIndex : activeAssigned(state)) {
            if (!state.positions[agentIndex].equals(state.parent.positions[agentIndex])) return false;
        }
        return true;
    }

    @Override
    public String name() {
        return type.name() + (lazyEvaluation ? "_LAZY_RDC" : "_RDC");
    }

    @Override
    public MuaHeuristicMetrics metrics() {
        return metrics;
    }

    /** MAX: max over active AAs of cap(ai), each computed by an independent single-AA capture search. */
    private long max(MuaSearchState state) {
        if (objective != MapfuaObjective.MUA) return independentDistanceLowerBound(state);
        long best = 0;
        for (int agentIndex : activeAssigned(state)) {
            int cap = minCaptureSearch(agentIndex, state.positions[agentIndex], state.movedUAs);
            if (cap == Integer.MAX_VALUE) return context.uaStartLocations.length + 1L;
            best = Math.max(best, cap);
        }
        return best;
    }

    /**
     * IH: try removing no UA, then every single unmoved UA, then every pair, etc. (deterministic
     * lexicographic order). Returns the first k for which some k-subset lets every active AA
     * independently reach its target.
     */
    private long increasing(MuaSearchState state) {
        if (objective != MapfuaObjective.MUA) return independentDistanceLowerBound(state);
        List<Integer> active = activeAssigned(state);
        if (active.isEmpty()) return 0;
        int[] uaIndexes = staticIndexes(context.staticUAs(state.movedUAs));
        for (int k = 0; k <= uaIndexes.length; k++) {
            if (someSubsetMakesAllReachable(state, active, uaIndexes, k, 0, new BitSet())) return k;
        }
        return context.uaStartLocations.length + 1L;
    }

    private boolean someSubsetMakesAllReachable(MuaSearchState state, List<Integer> active, int[] items, int k,
                                                int start, BitSet removed) {
        int chosen = removed.cardinality();
        if (chosen == k) {
            metrics.addInternalExpanded(1);
            return allAssignedReachable(state, active, removed);
        }
        for (int i = start; i <= items.length - (k - chosen); i++) {
            removed.set(items[i]);
            if (someSubsetMakesAllReachable(state, active, items, k, i + 1, removed)) return true;
            removed.clear(items[i]);
        }
        return false;
    }

    /**
     * MH: all active AAs merged into one meta-agent. Joint relaxed search (vertex/swap legal among
     * active AAs, reach-goal-once via the served bitset) minimizing distinct unmoved UAs captured.
     */
    private long merge(MuaSearchState state) {
        List<Integer> active = activeAssigned(state);
        if (active.isEmpty()) return 0;
        PriorityQueue<JointCANode> open = new PriorityQueue<>((a, b) -> {
            int byG = Long.compare(a.g, b.g);
            if (byG != 0) return byG;
            return Long.compare(a.id, b.id);
        });
        Map<JointCAKey, Long> best = new HashMap<>();
        I_Location[] start = new I_Location[active.size()];
        for (int i = 0; i < active.size(); i++) start[i] = state.positions[active.get(i)];
        long serial = 0;
        BitSet served = new BitSet(active.size());
        for (int i = 0; i < active.size(); i++) {
            if (start[i].getCoordinate().equals(context.agents.get(active.get(i)).target)) served.set(i);
        }
        JointCANode root = new JointCANode(start, new BitSet(), served, 0, serial++);
        open.add(root);
        best.put(root.key(), root.g);
        metrics.addInternalGenerated(1);
        BitSet staticUAs = context.staticUAs(state.movedUAs);
        while (!open.isEmpty()) {
            JointCANode current = open.poll();
            if (!Objects.equals(best.get(current.key()), current.g)) continue;
            metrics.addInternalExpanded(1);
            if (jointCAGoal(active, current.served)) return current.g;
            List<I_Location[]> moves = new ArrayList<>();
            enumerateJointCAMoves(active, current.positions, 0, new I_Location[active.size()], moves);
            for (I_Location[] next : moves) {
                BitSet captured = (BitSet) current.captured.clone();
                int newCaptures = 0;
                int fuel = 0;
                for (int i = 0; i < next.length; i++) {
                    if (!next[i].equals(current.positions[i])) fuel++;
                    int capture = capturedUAAt(next[i], staticUAs);
                    if (capture >= 0 && !captured.get(capture)) {
                        captured.set(capture);
                        newCaptures++;
                    }
                }
                BitSet childServed = (BitSet) current.served.clone();
                for (int i = 0; i < next.length; i++) {
                    if (next[i].getCoordinate().equals(context.agents.get(active.get(i)).target)) childServed.set(i);
                }
                long step = objective.stepCost(newCaptures, fuel + (objective == MapfuaObjective.FUEL ? newCaptures : 0),
                        current.served.cardinality() == active.size() ? 0 : unservedCount(active, current.served));
                JointCANode child = new JointCANode(next, captured, childServed, Math.addExact(current.g, step), serial++);
                JointCAKey key = child.key();
                Long old = best.get(key);
                if (old == null || child.g < old) {
                    best.put(key, child.g);
                    open.add(child);
                    metrics.addInternalGenerated(1);
                }
            }
        }
        return context.uaStartLocations.length + active.size() + 1L;
    }

    private long independentDistanceLowerBound(MuaSearchState state) {
        long distance = 0;
        BitSet staticUAs = context.staticUAs(state.movedUAs);
        for (int agentIndex : activeAssigned(state)) {
            distance = Math.addExact(distance, context.rdc.query(state.positions[agentIndex], context.targetOf(agentIndex), staticUAs));
        }
        return distance;
    }

    private int unservedCount(List<Integer> active, BitSet served) {
        return active.size() - served.cardinality();
    }

    private List<Integer> activeAssigned(MuaSearchState state) {
        List<Integer> active = new ArrayList<>();
        for (int agentIndex : context.assignedIndexes) {
            if (!state.visitedTargets.get(agentIndex)) active.add(agentIndex);
        }
        return active;
    }

    /** cap(ai): minimum number of distinct unmoved UAs ai must capture to reach its target (MAX_VALUE if unreachable). */
    private int minCaptureSearch(int agentIndex, I_Location start, BitSet movedUAs) {
        I_Location target = context.targetOf(agentIndex);
        BitSet staticUAs = context.staticUAs(movedUAs);
        PriorityQueue<CaptureNode> open = new PriorityQueue<>((a, b) -> {
            int byCaptures = Integer.compare(a.captures.cardinality(), b.captures.cardinality());
            if (byCaptures != 0) return byCaptures;
            int byFuel = Long.compare(a.fuel, b.fuel);
            if (byFuel != 0) return byFuel;
            return Long.compare(a.id, b.id);
        });
        Set<CaptureKey> closed = new HashSet<>();
        long serial = 0;
        open.add(new CaptureNode(start, new BitSet(), 0, serial++));
        metrics.addInternalGenerated(1);
        while (!open.isEmpty()) {
            CaptureNode n = open.poll();
            CaptureKey key = new CaptureKey(n.location, n.captures);
            if (!closed.add(key)) continue;
            metrics.addInternalExpanded(1);
            if (n.location.equals(target)) return n.captures.cardinality();
            for (I_Location next : orderedMoves(n.location, false)) {
                BitSet caps = (BitSet) n.captures.clone();
                int ua = capturedUAAt(next, staticUAs);
                if (ua >= 0) caps.set(ua);
                open.add(new CaptureNode(next, caps, n.fuel + (next.equals(n.location) ? 0 : 1), serial++));
                metrics.addInternalGenerated(1);
            }
        }
        return Integer.MAX_VALUE;
    }

    private boolean allAssignedReachable(MuaSearchState state, List<Integer> active, BitSet removedUAs) {
        BitSet obstacles = context.staticUAs(state.movedUAs);
        obstacles.andNot(removedUAs);
        for (int agentIndex : active) {
            if (!reachableAvoiding(state.positions[agentIndex], context.targetOf(agentIndex), obstacles)) return false;
        }
        return true;
    }

    private boolean reachableAvoiding(I_Location start, I_Location target, BitSet staticObstacles) {
        if (isUAObstacle(start, staticObstacles) || isUAObstacle(target, staticObstacles)) return false;
        ArrayDeque<I_Location> queue = new ArrayDeque<>();
        Set<I_Location> seen = new HashSet<>();
        seen.add(start);
        queue.add(start);
        while (!queue.isEmpty()) {
            I_Location current = queue.removeFirst();
            if (current.equals(target)) return true;
            for (I_Location next : current.outgoingEdges()) {
                if (isUAObstacle(next, staticObstacles) || !seen.add(next)) continue;
                queue.addLast(next);
            }
        }
        return false;
    }

    private boolean isUAObstacle(I_Location location, BitSet staticObstacles) {
        return capturedUAAt(location, staticObstacles) >= 0;
    }

    private int capturedUAAt(I_Location location, BitSet staticUAs) {
        for (int i = staticUAs.nextSetBit(0); i >= 0; i = staticUAs.nextSetBit(i + 1)) {
            if (context.uaStartLocations[i].equals(location)) return i;
        }
        return -1;
    }

    private void enumerateJointCAMoves(List<Integer> active, I_Location[] current, int index,
                                       I_Location[] chosen, List<I_Location[]> result) {
        if (index == active.size()) {
            result.add(Arrays.copyOf(chosen, chosen.length));
            return;
        }
        for (I_Location next : orderedMoves(current[index], true)) {
            boolean conflict = false;
            for (int j = 0; j < index; j++) {
                if (next.equals(chosen[j])) {
                    conflict = true;
                    break;
                }
                if (current[index].equals(chosen[j]) && next.equals(current[j])) {
                    conflict = true;
                    break;
                }
            }
            if (conflict) continue;
            chosen[index] = next;
            enumerateJointCAMoves(active, current, index + 1, chosen, result);
        }
    }

    private boolean jointCAGoal(List<Integer> active, BitSet served) {
        return served.cardinality() == active.size();
    }

    private List<I_Location> orderedMoves(I_Location from, boolean includeWait) {
        List<I_Location> moves = new ArrayList<>(from.outgoingEdges());
        moves.sort(Comparator.comparing(l -> l.getCoordinate().toString()));
        if (includeWait) moves.add(from);
        return moves;
    }

    private int[] staticIndexes(BitSet staticUAs) {
        int[] arr = new int[staticUAs.cardinality()];
        int p = 0;
        for (int i = staticUAs.nextSetBit(0); i >= 0; i = staticUAs.nextSetBit(i + 1)) arr[p++] = i;
        return arr;
    }

    private record CaptureKey(I_Location location, BitSet captures) {
        CaptureKey {
            captures = (BitSet) captures.clone();
        }
    }
    private record CaptureNode(I_Location location, BitSet captures, long fuel, long id) { }
    private record JointCAKey(I_Location[] positions, BitSet captured, BitSet served) {
        JointCAKey {
            positions = Arrays.copyOf(positions, positions.length);
            captured = (BitSet) captured.clone();
            served = (BitSet) served.clone();
        }
        @Override public boolean equals(Object o) {
            return o instanceof JointCAKey that
                    && Arrays.equals(positions, that.positions)
                    && captured.equals(that.captured)
                    && served.equals(that.served);
        }
        @Override public int hashCode() {
            int result = Arrays.hashCode(positions);
            result = 31 * result + captured.hashCode();
            result = 31 * result + served.hashCode();
            return result;
        }
    }
    private record JointCANode(I_Location[] positions, BitSet captured, BitSet served, long g, long id) {
        JointCAKey key() {
            return new JointCAKey(positions, captured, served);
        }
    }
}
