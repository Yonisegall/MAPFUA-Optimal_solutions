package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.Instances.Agent;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Immutable partition of all agents into CBS* meta-agents.
 */
public final class MetaAgentPartition {
    private final List<MetaAgent> metaAgents;
    private final List<Agent> allAgents;

    public MetaAgentPartition(Collection<MetaAgent> metaAgents, Collection<Agent> allAgents) {
        if (metaAgents == null || metaAgents.isEmpty() || allAgents == null || allAgents.isEmpty()) {
            throw new IllegalArgumentException("Partition and agent universe must be non-empty.");
        }
        ArrayList<Agent> sortedAgents = new ArrayList<>(allAgents);
        sortedAgents.sort((a, b) -> Integer.compare(a.iD, b.iD));
        this.allAgents = Collections.unmodifiableList(sortedAgents);

        Set<Agent> seen = new HashSet<>();
        ArrayList<MetaAgent> groups = new ArrayList<>(metaAgents);
        groups.sort((a, b) -> a.stableKey().compareTo(b.stableKey()));
        for (MetaAgent group : groups) {
            for (Agent agent : group.agents()) {
                if (!seen.add(agent)) {
                    throw new IllegalArgumentException("Agent appears in more than one meta-agent: " + agent);
                }
            }
        }
        if (!seen.containsAll(this.allAgents) || !this.allAgents.containsAll(seen)) {
            throw new IllegalArgumentException("Meta-agent partition is not complete.");
        }
        this.metaAgents = Collections.unmodifiableList(groups);
    }

    public static MetaAgentPartition singletonPartition(Collection<Agent> agents) {
        ArrayList<MetaAgent> groups = new ArrayList<>();
        for (Agent agent : agents) {
            groups.add(new MetaAgent(List.of(agent)));
        }
        return new MetaAgentPartition(groups, agents);
    }

    public List<MetaAgent> metaAgents() {
        return metaAgents;
    }

    public MetaAgent containing(Agent agent) {
        for (MetaAgent group : metaAgents) {
            if (group.contains(agent)) return group;
        }
        throw new IllegalArgumentException("Agent not present in partition: " + agent);
    }

    public MetaAgentPartition merge(Collection<MetaAgent> toMerge) {
        if (toMerge == null || toMerge.isEmpty()) return this;
        Set<MetaAgent> mergeSet = new HashSet<>(toMerge);
        ArrayList<MetaAgent> groups = new ArrayList<>();
        ArrayList<Agent> mergedAgents = new ArrayList<>();
        for (MetaAgent group : metaAgents) {
            if (mergeSet.contains(group)) {
                mergedAgents.addAll(group.agents());
            } else {
                groups.add(group);
            }
        }
        groups.add(new MetaAgent(mergedAgents));
        return new MetaAgentPartition(groups, allAgents);
    }

    public String stableKey() {
        StringBuilder sb = new StringBuilder();
        for (MetaAgent group : metaAgents) {
            if (sb.length() > 0) sb.append('|');
            sb.append(group.stableKey());
        }
        return sb.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof MetaAgentPartition that)) return false;
        return metaAgents.equals(that.metaAgents);
    }

    @Override
    public int hashCode() {
        return metaAgents.hashCode();
    }

    @Override
    public String toString() {
        return stableKey();
    }
}
