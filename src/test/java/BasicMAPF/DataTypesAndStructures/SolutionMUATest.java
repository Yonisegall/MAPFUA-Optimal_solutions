package BasicMAPF.DataTypesAndStructures;

import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.Maps.I_Location;
import org.junit.jupiter.api.Test;

import java.util.List;

import static BasicMAPF.TestConstants.Coordinates.*;
import static BasicMAPF.TestConstants.Maps.mapCircle;
import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * Ground-truth unit tests for the MUA objective, independent of any solver: MUA = number of DISTINCT unassigned
 * agents (UA, source.equals(target)) that moved (any non-wait move) at least once. A UA's contribution is capped at
 * 1 regardless of how many times it moves or whether it returns to its start; assigned agents (AA) never contribute.
 */
class SolutionMUATest {
    private final I_Location location12 = mapCircle.getMapLocation(coor12);
    private final I_Location location13 = mapCircle.getMapLocation(coor13);
    private final I_Location location14 = mapCircle.getMapLocation(coor14);
    private final I_Location location24 = mapCircle.getMapLocation(coor24);

    @Test
    void waitingUaContributesZeroToMUA() {
        Agent ua = new Agent(0, coor13, coor13);
        SingleAgentPlan waitOnly = new SingleAgentPlan(ua, List.of(
                new Move(ua, 1, location13, location13),
                new Move(ua, 2, location13, location13)));

        Solution solution = new Solution();
        solution.putPlan(waitOnly);

        assertEquals(0, solution.MUA());
        assertEquals(0, waitOnly.mua());
    }

    @Test
    void oneMoveMakesUaContributeOneToMUA() {
        Agent ua = new Agent(0, coor13, coor13);
        SingleAgentPlan onemove = new SingleAgentPlan(ua, List.of(new Move(ua, 1, location13, location14)));

        Solution solution = new Solution();
        solution.putPlan(onemove);

        assertEquals(1, solution.MUA());
        assertEquals(1, onemove.mua());
    }

    @Test
    void sameUaMovingMultipleTimesAndReturningStillContributesOnlyOne() {
        Agent ua = new Agent(0, coor13, coor13);
        SingleAgentPlan planLeavesAndComesBack = new SingleAgentPlan(ua, List.of(
                new Move(ua, 1, location13, location14),
                new Move(ua, 2, location14, location24),
                new Move(ua, 3, location24, location14),
                new Move(ua, 4, location14, location13)));

        Solution solution = new Solution();
        solution.putPlan(planLeavesAndComesBack);

        assertEquals(1, solution.MUA(), "a UA that moves repeatedly, even back to its own start, must still contribute exactly 1");
        assertEquals(1, planLeavesAndComesBack.mua());
    }

    @Test
    void twoDistinctUAsMovingContributeTwoToMUA() {
        Agent ua1 = new Agent(0, coor13, coor13);
        Agent ua2 = new Agent(1, coor12, coor12);
        SingleAgentPlan ua1Plan = new SingleAgentPlan(ua1, List.of(new Move(ua1, 1, location13, location14)));
        SingleAgentPlan ua2Plan = new SingleAgentPlan(ua2, List.of(new Move(ua2, 1, location12, location13)));

        Solution solution = new Solution();
        solution.putPlan(ua1Plan);
        solution.putPlan(ua2Plan);

        assertEquals(2, solution.MUA());
    }

    @Test
    void assignedAgentMovementNeverContributesToMUA() {
        Agent aa = new Agent(0, coor13, coor14);
        SingleAgentPlan aaPlan = new SingleAgentPlan(aa, List.of(new Move(aa, 1, location13, location14)));

        Solution solution = new Solution();
        solution.putPlan(aaPlan);

        assertEquals(0, solution.MUA(), "an assigned agent (source != target) must never contribute to MUA, however much it moves");
    }

    @Test
    void mixOfMovingAndWaitingUAsCountsOnlyThoseThatMoved() {
        Agent movingUa = new Agent(0, coor13, coor13);
        Agent waitingUa = new Agent(1, coor12, coor12);
        Agent aa = new Agent(2, coor14, coor24);

        SingleAgentPlan movingPlan = new SingleAgentPlan(movingUa, List.of(new Move(movingUa, 1, location13, location14)));
        SingleAgentPlan waitingPlan = new SingleAgentPlan(waitingUa, List.of(new Move(waitingUa, 1, location12, location12)));
        SingleAgentPlan aaPlan = new SingleAgentPlan(aa, List.of(new Move(aa, 1, location14, location24)));

        Solution solution = new Solution();
        solution.putPlan(movingPlan);
        solution.putPlan(waitingPlan);
        solution.putPlan(aaPlan);

        assertEquals(1, solution.MUA());
    }
}
