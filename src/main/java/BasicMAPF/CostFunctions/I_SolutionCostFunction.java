package BasicMAPF.CostFunctions;

import BasicMAPF.DataTypesAndStructures.Solution;
import Environment.Config;
import Environment.Metrics.InstanceReport;

public interface I_SolutionCostFunction {

    int solutionCost(Solution solution);

    String name();

    static void addCommonCostsToReport(Solution solution, InstanceReport report){
        report.putIntegerValue(MakespanServiceTime.NAME, MakespanServiceTime.instance.solutionCost(solution));
        report.putIntegerValue(SumOfCosts.NAME, SumOfCosts.instance.solutionCost(solution));
        report.putIntegerValue(Makespan.NAME, Makespan.instance.solutionCost(solution));
        // UA costs:
        report.putIntegerValue(SumServiceTimes.NAME, solution.SST());                   
        report.putIntegerValue(SumFuel.NAME, solution.FUEL());
        report.putIntegerValue(SumMUA.NAME, solution.MUA());
        report.putIntegerValue(FuelAssigned.NAME, solution.Fuel_Assigned());
        report.putIntegerValue(SumServiceTimesAll.NAME, solution.SST_All());

        if (Config.Misc.RECORD_SOLUTION_AGENT_COSTS_STRING){
            report.putStringValue(PathCosts.NAME, PathCosts.instance.getPathCostsString(solution));
            report.putStringValue(PathDelays.NAME, PathDelays.instance.getPathDelaysString(solution, report));
        }
    }

}
