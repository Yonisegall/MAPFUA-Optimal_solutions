package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Instances.Maps.I_Map;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Comparator;
import java.util.List;

final class MuaProblemContext {
    final MAPF_Instance instance;
    final I_Map map;
    final List<Agent> agents;
    final List<Integer> assignedIndexes = new ArrayList<>();
    final int[] uaIndexByAgentIndex;
    final I_Location[] uaStartLocations;
    final ReachabilityDistanceCache rdc;

    MuaProblemContext(MAPF_Instance instance) {
        this.instance = instance;
        this.map = instance.map;
        this.agents = new ArrayList<>(instance.agents);
        this.agents.sort(Comparator.comparingInt(a -> a.iD));
        this.uaIndexByAgentIndex = new int[agents.size()];
        List<I_Location> uaStarts = new ArrayList<>();
        for (int i = 0; i < agents.size(); i++) {
            Agent a = agents.get(i);
            uaIndexByAgentIndex[i] = -1;
            if (a.isUnassigned) {
                uaIndexByAgentIndex[i] = uaStarts.size();
                uaStarts.add(map.getMapLocation(a.source));
            } else {
                assignedIndexes.add(i);
            }
        }
        this.uaStartLocations = uaStarts.toArray(new I_Location[0]);
        this.rdc = new ReachabilityDistanceCache(map, uaStartLocations);
    }

    BitSet staticUAs(BitSet movedUAs) {
        BitSet staticUAs = new BitSet(uaStartLocations.length);
        staticUAs.set(0, uaStartLocations.length);
        staticUAs.andNot(movedUAs);
        return staticUAs;
    }

    I_Location targetOf(int agentIndex) {
        return map.getMapLocation(agents.get(agentIndex).target);
    }
}
