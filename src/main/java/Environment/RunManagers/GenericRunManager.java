package Environment.RunManagers;

import java.util.List;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.InstanceBuilders.I_InstanceBuilder;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Solvers.CanonicalSolversFactory;
import BasicMAPF.Solvers.I_Solver;
import Environment.Experiment;
import Environment.Visualization.I_VisualizeSolution;

public class GenericRunManager extends A_RunManager {

    private final String instancesDir;
    private final int[] agentNums;
    private final I_InstanceBuilder instanceBuilder;
    private final String experimentName;
    private final boolean skipAfterFail;
    private final String instancesRegex;
    private List<I_Solver> solversOverride;
    private final Integer timeoutEach;

    public GenericRunManager(@NotNull String instancesDir, int[] agentNums, @NotNull I_InstanceBuilder instanceBuilder,
                             @NotNull String experimentName, boolean skipAfterFail, String instancesRegex,
                             String resultsOutputDir, String resultsFilePrefix, I_VisualizeSolution solutionVisualizer,
                             Integer timeoutEach, @Nullable List<I_Solver> solversOverride) {
        super(resultsOutputDir, solutionVisualizer);
        if (agentNums == null){
            throw new IllegalArgumentException("AgentNums can't be null");
        }
        this.instancesDir = instancesDir;
        this.agentNums = agentNums;
        this.instanceBuilder = instanceBuilder;
        this.experimentName = experimentName;
        this.skipAfterFail = skipAfterFail;
        this.instancesRegex = instancesRegex;
        this.resultsFilePrefix = resultsFilePrefix;
        this.timeoutEach = timeoutEach;
        this.solversOverride = solversOverride;
    }
    @Override
    void setSolvers() {
        if (solversOverride != null){
            super.solvers = solversOverride;
            return;
        }

//───────────────────────────────────────────── Dynamic UA ─────────────────────────────────────────────

        // CBS:
        // super.solvers.add(CanonicalSolversFactory.createCBS_UA_SST_Solver());           // SST
        // super.solvers.add(CanonicalSolversFactory.createCBS_UA_Fuel_Solver());          // Fuel
        // super.solvers.add(CanonicalSolversFactory.createCBS_UA_MUA_Solver());           // MUA (Pure CBS)
        super.solvers.add(CanonicalSolversFactory.createCBSStarMUASolver());  // PURE MUA | CBS*
        // super.solvers.add(CanonicalSolversFactory.createCBSStarFuelSolver()); // PURE FUEL | CBS*
        // super.solvers.add(CanonicalSolversFactory.createCBSStarSSTSolver());  // PURE SST | CBS*

        // super.solvers.add(CanonicalSolversFactory.createMinMUACBSSolver());             // MUA (CBS with ID-LaCAM)
        // super.solvers.add(CanonicalSolversFactory.createCBS_UA_SST_ALL_Solver());       // SST All
        // super.solvers.add(CanonicalSolversFactory.createCBS_UA_Fuel_Assigned_Solver()); // FUEL Assigned

        // A*+OD:
        // super.solvers.add(CanonicalSolversFactory.createAStartOD_UA_SST_Solver());      // SST
        // super.solvers.add(CanonicalSolversFactory.createAStartOD_UA_Fuel_Solver());     // Fuel
        // super.solvers.add(CanonicalSolversFactory.createAStarOD_UA_MUA_Solver());       // MUA (AStar + OD)
        // super.solvers.add(CanonicalSolversFactory.createAStarODLaCAM_UA_MUA_Solver());  // MUA (LaCAM + AStar + OD)
        // A*OD - PURE MUA
        super.solvers.add(CanonicalSolversFactory.createAStarODMUAZeroSolver());          // PURE MUA | A*OD | h0 Zero
        super.solvers.add(CanonicalSolversFactory.createAStarODMUAH1MaxSolver());         // PURE MUA | A*OD | h1 Max
        super.solvers.add(CanonicalSolversFactory.createAStarODMUAH2EnhancedMaxSolver()); // PURE MUA | A*OD | h2 Enhanced Max
        super.solvers.add(CanonicalSolversFactory.createAStarODMUAH3JointSolver());       // PURE MUA | A*OD | h3 Joint
        super.solvers.add(CanonicalSolversFactory.createAStarODMUAH4RandomPickSolver());  // PURE MUA | A*OD | h4 Random Pick

        // A*OD - PURE FUEL
        // super.solvers.add(CanonicalSolversFactory.createAStarODFuelZeroSolver());          // PURE FUEL | A*OD | h0 Zero
        // super.solvers.add(CanonicalSolversFactory.createAStarODFuelH1MaxSolver());         // PURE FUEL | A*OD | h1 Max
        // super.solvers.add(CanonicalSolversFactory.createAStarODFuelH2EnhancedMaxSolver()); // PURE FUEL | A*OD | h2 Enhanced Max
        // super.solvers.add(CanonicalSolversFactory.createAStarODFuelH3JointSolver());       // PURE FUEL | A*OD | h3 Joint
        // super.solvers.add(CanonicalSolversFactory.createAStarODFuelH4RandomPickSolver());  // PURE FUEL | A*OD | h4 Random Pick

        // A*OD - PURE SST
        // super.solvers.add(CanonicalSolversFactory.createAStarODSSTZeroSolver());          // PURE SST | A*OD | h0 Zero
        // super.solvers.add(CanonicalSolversFactory.createAStarODSSTH1MaxSolver());         // PURE SST | A*OD | h1 Max
        // super.solvers.add(CanonicalSolversFactory.createAStarODSSTH2EnhancedMaxSolver()); // PURE SST | A*OD | h2 Enhanced Max
        // super.solvers.add(CanonicalSolversFactory.createAStarODSSTH3JointSolver());       // PURE SST | A*OD | h3 Joint
        // super.solvers.add(CanonicalSolversFactory.createAStarODSSTH4RandomPickSolver());  // PURE SST | A*OD | h4 Random Pick

        // LaCAM:
        // super.solvers.add(CanonicalSolversFactory.createLaCAMtSolver()); 

//───────────────────────────────────────────── Static UA ─────────────────────────────────────────────

        // CBS:
        // super.solvers.add(CanonicalSolversFactory.createCBS_SST_Solver());   // SST
        // super.solvers.add(CanonicalSolversFactory.createCBS_Fuel_Solver());  // Fuel

        // A*+OD:
        // super.solvers.add(CanonicalSolversFactory.createAStartOD_SST_Solver());   // SST
        // super.solvers.add(CanonicalSolversFactory.createAStartOD_Fuel_Solver());  // Fuel

        // LaCAM:
        // super.solvers.add(CanonicalSolversFactory.createLaCAMtStaticSolver());

//──────────────────────────────────────────────────────────────────────────────────────────────────────
    }
 
    public void overrideSolvers(@NotNull List<I_Solver> solvers){
        this.solversOverride = solvers;
    }

    @Override
    void setExperiments() {
        /*  =   Set Properties   =  */
        InstanceProperties properties = new InstanceProperties(null, -1, agentNums, instancesRegex);

        /*  =   Set Instance Manager   =  */
        InstanceManager instanceManager = new InstanceManager(instancesDir, instanceBuilder, properties);

        /*  =   Add new experiment   =  */
        Experiment experiment = new Experiment(experimentName, instanceManager, null, timeoutEach);
        experiment.skipAfterFail = this.skipAfterFail;
        experiment.visualizer = this.visualizer;
        this.experiments.add(experiment);
    }

    public static int countUnassignedAgents(MAPF_Instance instance) {
        int count = 0;
        for (Agent a : instance.agents) {
            if (a.isUnassigned) count++;
        }
        return count;
    }
}
