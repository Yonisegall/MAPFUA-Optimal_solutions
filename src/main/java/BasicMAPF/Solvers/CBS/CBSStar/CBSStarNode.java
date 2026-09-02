package BasicMAPF.Solvers.CBS.CBSStar;

import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;

/**
 * A CBS* CT node. Besides ordinary CBS constraints and paths, it stores the current meta-agent partition and the
 * meta-agent whose path was most recently recomputed; the paper's DetectJointLoop procedure is only run against that
 * group because all other paths are inherited unchanged from the parent node.
 */
public final class CBSStarNode {
    final ConstraintSet constraints;
    final Solution solution;
    final MetaAgentPartition partition;
    final MetaAgent newestMetaAgent;
    final int cost;
    final int depth;
    final long id;
    final boolean root;

    CBSStarNode(ConstraintSet constraints, Solution solution, MetaAgentPartition partition,
                MetaAgent newestMetaAgent, int cost, int depth, long id, boolean root) {
        this.constraints = new ConstraintSet(constraints);
        this.solution = new Solution(solution);
        this.partition = partition;
        this.newestMetaAgent = newestMetaAgent;
        this.cost = cost;
        this.depth = depth;
        this.id = id;
        this.root = root;
    }

    CBSStarNode childWith(Constraint addedConstraint, Solution childSolution, MetaAgent replanned, int childCost, long childId) {
        ConstraintSet childConstraints = new ConstraintSet(this.constraints);
        childConstraints.add(addedConstraint);
        return new CBSStarNode(childConstraints, childSolution, partition, replanned, childCost, depth + 1, childId, false);
    }

    CBSStarNode withMerge(MetaAgentPartition mergedPartition, MetaAgent mergedAgent, Solution mergedSolution,
                          int childCost, long childId) {
        return new CBSStarNode(this.constraints, mergedSolution, mergedPartition, mergedAgent, childCost, depth + 1, childId, false);
    }
}
