package BasicMAPF.Instances;

import BasicMAPF.Instances.Maps.Coordinates.Coordinate_2D;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class AgentTest {

    @Test
    void identifiesUnassignedAgentWhenSourceEqualsTarget() {
        Coordinate_2D location = new Coordinate_2D(1, 2);

        Agent agent = new Agent(1, location, new Coordinate_2D(1, 2));

        assertTrue(agent.isUnassigned);
    }

    @Test
    void identifiesAssignedAgentWhenSourceDiffersFromTarget() {
        Agent agent = new Agent(
                1,
                new Coordinate_2D(1, 2),
                new Coordinate_2D(2, 1),
                3
        );

        assertFalse(agent.isUnassigned);
    }
}
