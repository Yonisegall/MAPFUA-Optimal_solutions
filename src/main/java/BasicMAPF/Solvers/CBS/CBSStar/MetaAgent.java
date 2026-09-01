package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.Instances.Agent;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * A CBS* meta-agent: a non-empty deterministic set of agents that must be planned jointly.
 */
public final class MetaAgent {
    private final List<Agent> agents;

    public MetaAgent(Collection<Agent> agents) {
        if (agents == null || agents.isEmpty()) {
            throw new IllegalArgumentException("A meta-agent must contain at least one agent.");
        }
        ArrayList<Agent> sorted = new ArrayList<>(agents);
        sorted.sort((a, b) -> Integer.compare(a.iD, b.iD));
        for (int i = 1; i < sorted.size(); i++) {
            if (sorted.get(i - 1).equals(sorted.get(i))) {
                throw new IllegalArgumentException("Duplicate agent in meta-agent: " + sorted.get(i));
            }
        }
        this.agents = Collections.unmodifiableList(sorted);
    }

    public List<Agent> agents() {
        return agents;
    }

    public boolean contains(Agent agent) {
        return agents.contains(agent);
    }

    public int size() {
        return agents.size();
    }

    public MetaAgent merge(MetaAgent other) {
        ArrayList<Agent> merged = new ArrayList<>(this.agents);
        merged.addAll(other.agents);
        return new MetaAgent(merged);
    }

    public String stableKey() {
        StringBuilder sb = new StringBuilder();
        for (Agent agent : agents) {
            if (sb.length() > 0) sb.append(',');
            sb.append(agent.iD);
        }
        return sb.toString();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof MetaAgent that)) return false;
        return agents.equals(that.agents);
    }

    @Override
    public int hashCode() {
        return Objects.hash(agents);
    }

    @Override
    public String toString() {
        return "{" + stableKey() + "}";
    }
}
