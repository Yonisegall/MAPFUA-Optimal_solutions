package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Instances.Maps.I_Map;

import java.util.ArrayDeque;
import java.util.BitSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

final class ReachabilityDistanceCache {
    private final I_Map map;
    private final I_Location[] uaStartLocations;
    private final Map<Key, Map<I_Location, Integer>> cache = new HashMap<>();
    long builds;
    long queries;
    long hits;
    long misses;
    long manhattanFallbacks;

    ReachabilityDistanceCache(I_Map map, I_Location[] uaStartLocations) {
        this.map = map;
        this.uaStartLocations = uaStartLocations;
    }

    int query(I_Location from, I_Location target, BitSet staticUAs) {
        queries++;
        Key key = new Key(target, staticUAs);
        Map<I_Location, Integer> distances = cache.get(key);
        if (distances == null) {
            misses++;
            distances = build(target, staticUAs);
            cache.put(key, distances);
        } else {
            hits++;
        }
        Integer exact = distances.get(from);
        if (exact != null) return exact;
        manhattanFallbacks++;
        int fallback = (int) from.getCoordinate().distance(target.getCoordinate());
        return Math.max(fallback, 0);
    }

    boolean reachableWithoutFallback(I_Location from, I_Location target, BitSet staticUAs) {
        Key key = new Key(target, staticUAs);
        Map<I_Location, Integer> distances = cache.get(key);
        if (distances == null) {
            misses++;
            distances = build(target, staticUAs);
            cache.put(key, distances);
        } else {
            hits++;
        }
        return distances.containsKey(from);
    }

    private Map<I_Location, Integer> build(I_Location target, BitSet staticUAs) {
        builds++;
        Set<I_Location> obstacles = staticLocations(staticUAs);
        Map<I_Location, Integer> distances = new HashMap<>();
        if (obstacles.contains(target)) {
            return distances;
        }
        ArrayDeque<I_Location> queue = new ArrayDeque<>();
        distances.put(target, 0);
        queue.add(target);
        while (!queue.isEmpty()) {
            I_Location current = queue.removeFirst();
            int nextDistance = distances.get(current) + 1;
            for (I_Location prev : current.incomingEdges()) {
                if (obstacles.contains(prev) || distances.containsKey(prev)) continue;
                distances.put(prev, nextDistance);
                queue.addLast(prev);
            }
        }
        return distances;
    }

    private Set<I_Location> staticLocations(BitSet staticUAs) {
        Set<I_Location> blocked = new HashSet<>();
        for (int i = staticUAs.nextSetBit(0); i >= 0; i = staticUAs.nextSetBit(i + 1)) {
            blocked.add(uaStartLocations[i]);
        }
        return blocked;
    }

    private static final class Key {
        private final I_Location target;
        private final BitSet staticUAs;

        private Key(I_Location target, BitSet staticUAs) {
            this.target = target;
            this.staticUAs = (BitSet) staticUAs.clone();
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Key key)) return false;
            return target.equals(key.target) && staticUAs.equals(key.staticUAs);
        }

        @Override
        public int hashCode() {
            return Objects.hash(target, staticUAs);
        }
    }
}
