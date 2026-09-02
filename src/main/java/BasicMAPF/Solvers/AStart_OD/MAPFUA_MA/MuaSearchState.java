package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.Instances.Maps.I_Location;

import java.util.Arrays;
import java.util.BitSet;
import java.util.Objects;

final class MuaSearchState {
    final int time;
    final I_Location[] positions;
    final BitSet movedUAs;
    final BitSet visitedTargets;
    final boolean partial;
    final int nextAgent;
    final I_Location[] nextPositions;
    final BitSet nextMovedUAs;
    final BitSet nextVisitedTargets;
    final Move[] chosenMoves;
    final long g;
    final long h;
    final MuaSearchState parent;
    final Move[] jointMovesFromParent;
    final long id;
    final int stepFuel;
    final int stepNewUA;

    private MuaSearchState(int time, I_Location[] positions, BitSet movedUAs, BitSet visitedTargets,
                           boolean partial, int nextAgent, I_Location[] nextPositions, BitSet nextMovedUAs,
                           BitSet nextVisitedTargets, Move[] chosenMoves, long g, long h,
                           MuaSearchState parent, Move[] jointMovesFromParent, long id, int stepFuel, int stepNewUA) {
        this.time = time;
        this.positions = positions;
        this.movedUAs = (BitSet) movedUAs.clone();
        this.visitedTargets = (BitSet) visitedTargets.clone();
        this.partial = partial;
        this.nextAgent = nextAgent;
        this.nextPositions = nextPositions;
        this.nextMovedUAs = nextMovedUAs == null ? null : (BitSet) nextMovedUAs.clone();
        this.nextVisitedTargets = nextVisitedTargets == null ? null : (BitSet) nextVisitedTargets.clone();
        this.chosenMoves = chosenMoves;
        this.g = g;
        this.h = h;
        this.parent = parent;
        this.jointMovesFromParent = jointMovesFromParent;
        this.id = id;
        this.stepFuel = stepFuel;
        this.stepNewUA = stepNewUA;
    }

    static MuaSearchState full(int time, I_Location[] positions, BitSet movedUAs, BitSet visitedTargets,
                               long g, long h, MuaSearchState parent, Move[] jointMoves, long id) {
        return new MuaSearchState(time, positions, movedUAs, visitedTargets, false, -1, null, null, null,
                null, g, h, parent, jointMoves, id, 0, 0);
    }

    static MuaSearchState partial(MuaSearchState base, int nextAgent, I_Location[] nextPositions,
                                  BitSet nextMovedUAs, BitSet nextVisitedTargets, Move[] chosenMoves,
                                  long h, long id, int stepFuel, int stepNewUA) {
        return new MuaSearchState(base.time, base.positions, base.movedUAs, base.visitedTargets, true,
                nextAgent, nextPositions, nextMovedUAs, nextVisitedTargets, chosenMoves, base.g, h,
                base, null, id, stepFuel, stepNewUA);
    }

    long f() {
        return Math.addExact(g, h);
    }

    MuaStateKey key() {
        return partial
                ? MuaStateKey.partial(positions, movedUAs, visitedTargets, nextPositions, nextMovedUAs, nextVisitedTargets, nextAgent)
                : MuaStateKey.full(positions, movedUAs, visitedTargets);
    }

    MuaSearchState fullParent() {
        MuaSearchState n = this;
        while (n != null && n.partial) {
            n = n.parent;
        }
        return n == this ? parent : n;
    }

    static final class MuaStateKey {
        private final boolean partial;
        private final int nextAgent;
        private final I_Location[] positions;
        private final BitSet movedUAs;
        private final BitSet visitedTargets;
        private final I_Location[] nextPositionsPrefix;
        private final BitSet nextMovedUAs;
        private final BitSet nextVisitedTargets;

        private MuaStateKey(boolean partial, int nextAgent, I_Location[] positions, BitSet movedUAs,
                            BitSet visitedTargets, I_Location[] nextPositionsPrefix, BitSet nextMovedUAs,
                            BitSet nextVisitedTargets) {
            this.partial = partial;
            this.nextAgent = nextAgent;
            this.positions = Arrays.copyOf(positions, positions.length);
            this.movedUAs = (BitSet) movedUAs.clone();
            this.visitedTargets = (BitSet) visitedTargets.clone();
            this.nextPositionsPrefix = nextPositionsPrefix == null ? null : Arrays.copyOf(nextPositionsPrefix, nextPositionsPrefix.length);
            this.nextMovedUAs = nextMovedUAs == null ? null : (BitSet) nextMovedUAs.clone();
            this.nextVisitedTargets = nextVisitedTargets == null ? null : (BitSet) nextVisitedTargets.clone();
        }

        static MuaStateKey full(I_Location[] positions, BitSet movedUAs, BitSet visitedTargets) {
            return new MuaStateKey(false, -1, positions, movedUAs, visitedTargets, null, null, null);
        }

        static MuaStateKey partial(I_Location[] positions, BitSet movedUAs, BitSet visitedTargets,
                                   I_Location[] nextPositions, BitSet nextMovedUAs,
                                   BitSet nextVisitedTargets, int nextAgent) {
            I_Location[] prefix = new I_Location[nextAgent];
            System.arraycopy(nextPositions, 0, prefix, 0, nextAgent);
            return new MuaStateKey(true, nextAgent, positions, movedUAs, visitedTargets,
                    prefix, nextMovedUAs, nextVisitedTargets);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof MuaStateKey that)) return false;
            return partial == that.partial
                    && nextAgent == that.nextAgent
                    && Arrays.equals(positions, that.positions)
                    && Objects.equals(movedUAs, that.movedUAs)
                    && Objects.equals(visitedTargets, that.visitedTargets)
                    && Arrays.equals(nextPositionsPrefix, that.nextPositionsPrefix)
                    && Objects.equals(nextMovedUAs, that.nextMovedUAs)
                    && Objects.equals(nextVisitedTargets, that.nextVisitedTargets);
        }

        @Override
        public int hashCode() {
            int result = Objects.hash(partial, nextAgent, movedUAs, visitedTargets, nextMovedUAs, nextVisitedTargets);
            result = 31 * result + Arrays.hashCode(positions);
            result = 31 * result + Arrays.hashCode(nextPositionsPrefix);
            return result;
        }
    }
}
