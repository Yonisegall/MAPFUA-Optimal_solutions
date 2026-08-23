package BasicMAPF.CostFunctions;

import BasicMAPF.DataTypesAndStructures.Solution;

/**
 * MUA-only cost function.
 *
 * Objective:
 *   Minimize only the number of unassigned agents that moved at least once.
 *
 * Important:
 *   This intentionally does NOT include FUEL, SST, makespan, or any other secondary objective.
 *   CBS may still use deterministic tie-breaking between nodes with the same MUA value,
 *   but that tie-breaking is not part of the research objective.
 */
public class SumMUA implements I_SolutionCostFunction {

    public static final String NAME = "MUA";
    public static final SumMUA instance = new SumMUA();

    // @Override
    // public int solutionCost(Solution solution) {
    //     return solution.MUA();
    // }

    @Override
    public int solutionCost(Solution solution) {
        int mua = solution.MUA();
        // System.out.println("[SumMUA] solutionCost = " + mua);
        return mua;
    }

    @Override
    public String name() {
        return NAME;
    }
}