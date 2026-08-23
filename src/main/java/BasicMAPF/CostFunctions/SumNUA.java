package BasicMAPF.CostFunctions;

import BasicMAPF.DataTypesAndStructures.Solution;

/**
 * NUA-only cost function.
 *
 * Objective:
 *   Minimize only the number of unassigned agents that moved at least once.
 *
 * Important:
 *   This intentionally does NOT include FUEL, SST, makespan, or any other secondary objective.
 *   CBS may still use deterministic tie-breaking between nodes with the same NUA value,
 *   but that tie-breaking is not part of the research objective.
 */
public class SumNUA implements I_SolutionCostFunction {

    public static final String NAME = "NUA";
    public static final SumNUA instance = new SumNUA();

    // @Override
    // public int solutionCost(Solution solution) {
    //     return solution.NUA();
    // }

    @Override
    public int solutionCost(Solution solution) {
        int nua = solution.NUA();
        // System.out.println("[SumNUA] solutionCost = " + nua);
        return nua;
    }

    @Override
    public String name() {
        return NAME;
    }
}