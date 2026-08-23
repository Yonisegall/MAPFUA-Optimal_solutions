package BasicMAPF.Solvers.LaCAM;

import java.util.Set;

import BasicMAPF.CostFunctions.I_SolutionCostFunction;
import BasicMAPF.Instances.Agent;
import TransientMAPF.TransientMAPFSettings;

public class LaCAMBuilder {
    private I_SolutionCostFunction solutionCostFunction = null;
    private TransientMAPFSettings transientMAPFSettings = null;
    private boolean staticObstaclesForUnassignedAgents = false;

    // new fields for static UA behaviour
    private boolean staticUAForUnassignedAgents = false;
    private Set<Agent> staticUA = null;

    ////////////////////////////////////////////////////////////////
    private int muaBudget = -1;
    ////////////////////////////////////////////////////////////////

    public LaCAMBuilder setSolutionCostFunction(I_SolutionCostFunction solutionCostFunction) {
        this.solutionCostFunction = solutionCostFunction;
        return this;
    }

    public LaCAMBuilder setTransientMAPFBehaviour(TransientMAPFSettings transientMAPFSettings) {
        this.transientMAPFSettings = transientMAPFSettings;
        return this;
    }

    public LaCAMBuilder setStaticObstaclesForUnassignedAgents(boolean staticObstaclesForUnassignedAgents) {
        this.staticObstaclesForUnassignedAgents = staticObstaclesForUnassignedAgents;
        return this;
    }

    public LaCAMBuilder setStaticUAForUnassignedAgents(boolean staticUAForUnassignedAgents) {
        this.staticUAForUnassignedAgents = staticUAForUnassignedAgents;
        return this;
    }

    public LaCAMBuilder setStaticUA(Set<Agent> staticUA) {
        this.staticUA = staticUA;
        return this;
    }

    /////////////////////////////////////////////////////////////////////////////////
    public LaCAMBuilder setMUABudget(int budget) {
        this.muaBudget = budget;
        return this;
    }
    /////////////////////////////////////////////////////////////////////////////////

    // public LaCAM_Solver createLaCAM() {
    //     LaCAM_Solver solver = new LaCAM_Solver(solutionCostFunction, transientMAPFSettings, staticObstaclesForUnassignedAgents || staticUAForUnassignedAgents);
    //     if (this.staticUAForUnassignedAgents) {
    //         solver.setStaticUAForUnassignedAgents(true);
    //         solver.setStaticUA(staticUA);
    //     }
    //     return solver;
    // }

    public LaCAM_Solver createLaCAM() {
        LaCAM_Solver solver = new LaCAM_Solver(solutionCostFunction, transientMAPFSettings, staticObstaclesForUnassignedAgents || staticUAForUnassignedAgents);
        solver.setStaticUAForUnassignedAgents(this.staticUAForUnassignedAgents);
        solver.setStaticUA(this.staticUA);
        solver.muaBudget = this.muaBudget;
        return solver;
    }
}



// package BasicMAPF.Solvers.LaCAM;

// import BasicMAPF.CostFunctions.I_SolutionCostFunction;
// import TransientMAPF.TransientMAPFSettings;

// public class LaCAMBuilder {
//     private I_SolutionCostFunction solutionCostFunction = null;
//     private TransientMAPFSettings transientMAPFSettings = null;
//     private boolean staticObstaclesForUnassignedAgents = false;

//     public LaCAMBuilder setSolutionCostFunction(I_SolutionCostFunction solutionCostFunction) {
//         this.solutionCostFunction = solutionCostFunction;
//         return this;
//     }

//     public LaCAMBuilder setTransientMAPFBehaviour(TransientMAPFSettings transientMAPFSettings) {
//         this.transientMAPFSettings = transientMAPFSettings;
//         return this;
//     }

//     public LaCAMBuilder setStaticObstaclesForUnassignedAgents(boolean staticObstaclesForUnassignedAgents) {
//         this.staticObstaclesForUnassignedAgents = staticObstaclesForUnassignedAgents;
//         return this;
//     }

//     public LaCAM_Solver createLaCAM() {
//         return new LaCAM_Solver(solutionCostFunction, transientMAPFSettings, staticObstaclesForUnassignedAgents);
//     }
// }
