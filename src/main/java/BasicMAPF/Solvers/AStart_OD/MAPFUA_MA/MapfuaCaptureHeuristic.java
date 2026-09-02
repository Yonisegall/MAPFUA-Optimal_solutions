package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.Instances.Agent;
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
import java.util.Random;
import java.util.Set;

final class MapfuaCaptureHeuristic implements MuaHeuristic {
    private final MuaProblemContext context;
    private final MapfuaObjective objective;
    private final MuaHeuristicType type;
    private final boolean lazyEvaluation;
    private final MuaHeuristicMetrics metrics = new MuaHeuristicMetrics();
    private final Random random = new Random(4);

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
            case MAX -> h1(state);
            case ENHANCED_MAX -> h2(state);
            case JOINT_CAPTURE -> h3(state);
            case RANDOM_PICK -> h4(state);
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

    private long h1(MuaSearchState state) {
        if (objective != MapfuaObjective.MUA) return independentDistanceLowerBound(state);
        long best = 0;
        for (int agentIndex : activeAssigned(state)) {
            CaptureSearchResult r = minCaptureSearch(agentIndex, state.positions[agentIndex], state.movedUAs, false);
            if (r.minCaptures == Integer.MAX_VALUE) return context.uaStartLocations.length + 1L;
            best = Math.max(best, r.minCaptures);
        }
        return best;
    }

    private long h2(MuaSearchState state) {
        if (objective != MapfuaObjective.MUA) return independentDistanceLowerBound(state);
        List<Integer> active = activeAssigned(state);
        if (active.isEmpty()) return 0;
        int k = 0;
        Map<Integer, CaptureSearchResult> perAgent = new HashMap<>();
        for (int agentIndex : active) {
            CaptureSearchResult r = minCaptureSearch(agentIndex, state.positions[agentIndex], state.movedUAs, true);
            perAgent.put(agentIndex, r);
            if (r.minCaptures == Integer.MAX_VALUE) return context.uaStartLocations.length + 1L;
            k = Math.max(k, r.minCaptures);
        }
        boolean helpful = false;
        for (int agentIndex : active) {
            CaptureSearchResult r = perAgent.get(agentIndex);
            if (r.minCaptures != k) continue;
            for (BitSet set : r.captureSets) {
                if (allAssignedReachable(state, set)) {
                    helpful = true;
                    break;
                }
            }
            if (helpful) break;
        }
        return helpful ? k : k + 1L;
    }

    private long h3(MuaSearchState state) {
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

    private long h4(MuaSearchState state) {
        if (objective != MapfuaObjective.MUA) return independentDistanceLowerBound(state);
        BitSet staticUAs = context.staticUAs(state.movedUAs);
        int[] uaIndexes = staticIndexes(staticUAs);
        for (int k = 0; k <= uaIndexes.length; k++) {
            List<BitSet> sets = new ArrayList<>();
            enumerateCombinations(uaIndexes, k, 0, new BitSet(), sets);
            deterministicShuffle(sets);
            for (BitSet set : sets) {
                if (allAssignedReachable(state, set)) {
                    return k;
                }
            }
        }
        return context.uaStartLocations.length + 1L;
    }

    private void deterministicShuffle(List<BitSet> sets) {
        for (int i = sets.size() - 1; i > 0; i--) {
            int j = random.nextInt(i + 1);
            BitSet tmp = sets.get(i);
            sets.set(i, sets.get(j));
            sets.set(j, tmp);
        }
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

    private CaptureSearchResult minCaptureSearch(int agentIndex, I_Location start, BitSet movedUAs, boolean collectSets) {
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
        List<BitSet> sets = new ArrayList<>();
        int bestK = Integer.MAX_VALUE;
        long serial = 0;
        open.add(new CaptureNode(start, new BitSet(), 0, serial++));
        metrics.addInternalGenerated(1);
        while (!open.isEmpty()) {
            CaptureNode n = open.poll();
            CaptureKey key = new CaptureKey(n.location, n.captures);
            if (!closed.add(key)) continue;
            metrics.addInternalExpanded(1);
            int k = n.captures.cardinality();
            if (bestK != Integer.MAX_VALUE && k > bestK) break;
            if (n.location.equals(target)) {
                bestK = k;
                if (!collectSets) return new CaptureSearchResult(k, List.of((BitSet) n.captures.clone()));
                sets.add((BitSet) n.captures.clone());
                continue;
            }
            for (I_Location next : orderedMoves(n.location, false)) {
                BitSet caps = (BitSet) n.captures.clone();
                int ua = capturedUAAt(next, staticUAs);
                if (ua >= 0) caps.set(ua);
                if (bestK != Integer.MAX_VALUE && caps.cardinality() > bestK) continue;
                open.add(new CaptureNode(next, caps, n.fuel + (next.equals(n.location) ? 0 : 1), serial++));
                metrics.addInternalGenerated(1);
            }
        }
        return new CaptureSearchResult(bestK, dedupeSets(sets));
    }

    private boolean allAssignedReachable(MuaSearchState state, BitSet removedUAs) {
        BitSet obstacles = context.staticUAs(state.movedUAs);
        obstacles.andNot(removedUAs);
        for (int agentIndex : activeAssigned(state)) {
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

    private void enumerateCombinations(int[] items, int k, int start, BitSet current, List<BitSet> result) {
        if (current.cardinality() == k) {
            result.add((BitSet) current.clone());
            return;
        }
        for (int i = start; i < items.length; i++) {
            current.set(items[i]);
            enumerateCombinations(items, k, i + 1, current, result);
            current.clear(items[i]);
        }
    }

    private List<BitSet> dedupeSets(List<BitSet> sets) {
        Map<BitSet, BitSet> unique = new HashMap<>();
        for (BitSet set : sets) unique.put((BitSet) set.clone(), set);
        return new ArrayList<>(unique.keySet());
    }

    private record CaptureSearchResult(int minCaptures, List<BitSet> captureSets) { }
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
