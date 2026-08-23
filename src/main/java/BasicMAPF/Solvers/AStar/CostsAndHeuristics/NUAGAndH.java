package BasicMAPF.Solvers.AStar.CostsAndHeuristics;

import org.jetbrains.annotations.NotNull;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.Instances.Maps.Coordinates.I_Coordinate;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.AStar.SingleAgentAStar_Solver;

/**
 * A {@link SingleAgentGAndH} that wraps another implementation for NUA (Non-Unit Acceleration) cost function.
 * 
 * Assigned Agent (before reaching target): cost = 1, heuristic = distance
 * Unassigned Agent (after reaching target):
 *   - First move after target = 100
 *   - Subsequent moves/waits = 0
 *   - Heuristic = 0
 */
public class NUAGAndH implements SingleAgentGAndH {

    private final SingleAgentGAndH gAndH;

    /**
     * Constructor.
     * @param gAndH delegate to this {@link SingleAgentGAndH}
     */
    public NUAGAndH(@NotNull SingleAgentGAndH gAndH) {
        this.gAndH = gAndH;
    }

    @Override
    public int getHToTargetFromLocation(@NotNull I_Coordinate target, @NotNull I_Location currLocation) {
        return gAndH.getHToTargetFromLocation(target, currLocation);
    }

    @Override
    public boolean isConsistent() {
        return gAndH.isConsistent();
    }

    @Override
    public float getH(@NotNull SingleAgentAStar_Solver.AStarState state) {
        // Unassigned (after target): heuristic = 0
        if (state.visitedTarget) {return 0;}
        
        // Assigned (before target): heuristic = distance
        return getHToTargetFromLocation(state.getMove().agent.target, state.getMove().currLocation);
    }

    /**
     * NUA cost:
     * - Assigned: cost = 1
     * - Unassigned (after target):
     *     - First move: cost = 1
     *     - Subsequent: cost = 0
     */
    @Override
    public int cost(Move move, boolean isAfterTargetExcludingFirstMoveToTarget) {
        if (!isAfterTargetExcludingFirstMoveToTarget) {
            return 1;
        }

        I_Coordinate target = move.agent.target;

        boolean isWait = move.prevLocation.equals(move.currLocation);
        boolean prevIsTarget = move.prevLocation.getCoordinate().equals(target);

        if (isWait && prevIsTarget) {
            return 0;
        }

        if (!isWait && prevIsTarget) {
            return 1;
        }

        return 1;
    }
    

    @Override
    // Default: Assigned agent cost = 1
    public int cost(Move move) {return 1;}

    /**
     * Override to check if this is the first move after target reached
     */
    @Override
    public int cost(Move move, SingleAgentAStar_Solver.AStarState prevState) {
        boolean afterTarget = prevState != null && prevState.visitedTarget;

        /*
        * Before the target is first visited:
        * This is normal path finding for assigned agents.
        * We use step cost 1 only as a low-level search mechanism.
        */
        if (!afterTarget) {
            return 1;
        }

        /*
        * After target was already visited.
        * For UA, source == target, so this is true from t=0.
        */

        I_Coordinate target = move.agent.target;

        boolean isWait = move.prevLocation.equals(move.currLocation);
        boolean prevIsTarget = move.prevLocation.getCoordinate().equals(target);

        /*
        * UA waits at its original source/target:
        * No UA movement happened yet.
        */
        if (isWait && prevIsTarget) {
            return 0;
        }

        /*
        * First move away from source/target:
        * This activates this UA.
        */
        if (!isWait && prevIsTarget) {
            return 1;
        }

        /*
        * The UA has already moved before.
        *
        * This is NOT part of the high-level NUA objective.
        * It is only a low-level technical tie-break so A* prefers shorter,
        * finite paths instead of wandering/waiting forever with cost 0.
        */
        return 1;
    }

    public SingleAgentGAndH getWrappedHeuristic() {return this.gAndH;}

    @Override
    public boolean isTransient() {return true;}
}