package BasicMAPF.Solvers.CBS;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Queue;
import java.util.Set;

import org.jetbrains.annotations.Nullable;

import BasicMAPF.CostFunctions.I_SolutionCostFunction;
import BasicMAPF.CostFunctions.SumFuel;
import BasicMAPF.CostFunctions.SumNUA;
import BasicMAPF.CostFunctions.SumOfCosts;
import BasicMAPF.CostFunctions.SumServiceTimes;
import BasicMAPF.DataTypesAndStructures.I_OpenList;
import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.OpenListHeap;
import BasicMAPF.DataTypesAndStructures.RunParameters;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.Coordinates.I_Coordinate;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.CachingDistanceTableHeuristic;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.DistanceTableSingleAgentHeuristic;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.FuelGAndH;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.NUAGAndH;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.ServiceTimeGAndH;
import BasicMAPF.Solvers.AStar.CostsAndHeuristics.SingleAgentGAndH;
import BasicMAPF.Solvers.AStar.RunParameters_SAAStar;
import BasicMAPF.Solvers.AStar.SingleAgentAStar_Solver;
import BasicMAPF.Solvers.A_Solver;
import BasicMAPF.Solvers.ConstraintsAndConflicts.A_Conflict;
import BasicMAPF.Solvers.ConstraintsAndConflicts.ConflictManagement.ConflictAvoidance.I_ConflictAvoidanceTable;
import BasicMAPF.Solvers.ConstraintsAndConflicts.ConflictManagement.ConflictAvoidance.RemovableConflictAvoidanceTableWithContestedGoals;
import BasicMAPF.Solvers.ConstraintsAndConflicts.ConflictManagement.ConflictAvoidance.SingleUseConflictAvoidanceTable;
import BasicMAPF.Solvers.ConstraintsAndConflicts.ConflictManagement.ConflictManager;
import BasicMAPF.Solvers.ConstraintsAndConflicts.ConflictManagement.CorridorConflictManager;
import BasicMAPF.Solvers.ConstraintsAndConflicts.ConflictManagement.I_ConflictManager;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.Constraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.ConstraintSet;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.GoalConstraint;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.I_ConstraintSet;
import BasicMAPF.Solvers.ConstraintsAndConflicts.Constraint.UnmodifiableConstraintSet;
import BasicMAPF.Solvers.I_Solver;
import Environment.Config;
import Environment.Metrics.InstanceReport;
import TransientMAPF.TransientMAPFSettings;
import TransientMAPF.TransientMAPFSolution;
import TransientMAPF.TransientMAPFUtils;

/**
 * The Conflict Based Search (CBS) Multi Agent Path Finding (MAPF) algorithm.
 */
public class CBS_Solver extends A_Solver {

    /*  = Fields =  */

    /*  = Fields related to the MAPF instance =  */

    private MAPF_Instance instance;

    /*  = Fields related to the run =  */

    /**
     * A {@link SingleAgentGAndH heuristic} for the low level solver.
     */
    private SingleAgentGAndH singleAgentGAndH;
    /**
     * Initial constraints given to the solver to work with.
     */
    private I_ConstraintSet initialConstraints;
    /**
     * Reused for each {@link CBS_Node}.
     */
    private I_ConstraintSet currentConstraints;

    /*  = Fields related to the class instance =  */

    /**
     * A queue of open {@link CBS_Node nodes/states}. Also referred to as OPEN.
     */
    public final I_OpenList<CBS_Node> openList;
    /**
     * @see OpenListManagementMode
     */
    private final OpenListManagementMode openListManagementMode;
    /**
     * A {@link I_Solver solver}, to be used for solving single-{@link Agent agent} sub-problems.
     */
    private final I_Solver lowLevelSolver;
    /**
     * Cost may be more complicated than a simple SOC (Sum of Individual Costs), so retrieve it through this method.
     */
    private final I_SolutionCostFunction costFunction;
    /**
     * Determines how to sort {@link #openList OPEN}.
     */
    private final Comparator<? super CBS_Node> CBSNodeComparator;
    /**
     * Whether to use corridor reasoning.
     * @see <a href="jiaoyangli.me/files/2020-ICAPS.pdf#page=1&zoom=180,-78,792">New Techniques for Pairwise Symmetry Breaking in Multi-Agent Path Finding</a>
     */
    private final boolean corridorReasoning;
    /**
     * if true, agents can have shared goals, so they can stay at their goal together (only last move onwards).
     */
    private final boolean sharedGoals;

    /**
     * If true, agents staying at their source (since the start) will not conflict 
     */
    private final boolean sharedSources;
    private Set<I_Coordinate> separatingVerticesSet;
    private final boolean staticObstaclesForUnassignedAgents;
    private List<Agent> unassignedAgents = null;
    private Set<Agent> staticUA = null;
    private Solution initialSolution = null;
    private Set<String> seenConstraintSignatures = new HashSet<>();
    private Set<Integer> forcedMovedUAIds = new HashSet<>();

    /*  = Constructors =  */

    /**
     * Parameterised constructor.
     * @param lowLevelSolver this {@link I_Solver solver} will be used to solve single agent sub-problems.
     * @param openList this will be used as the {@link I_OpenList open list} in the solver. This instance will be reused by calling {@link I_OpenList#clear()} after every run.
     * @param openListManagementMode @see {@link OpenListManagementMode}.
     * @param costFunction a cost function for solutions.
     * @param cbsNodeComparator determines how to sort {@link #openList OPEN}.
     * @param useCorridorReasoning whether to use corridor reasoning.
     */

    CBS_Solver(@Nullable I_Solver lowLevelSolver, @Nullable I_OpenList<CBS_Node> openList, @Nullable OpenListManagementMode openListManagementMode,
               @Nullable I_SolutionCostFunction costFunction, @Nullable Comparator<? super CBS_Node> cbsNodeComparator, @Nullable Boolean useCorridorReasoning,
               @Nullable Boolean sharedGoals, @Nullable Boolean sharedSources, @Nullable TransientMAPFSettings transientMAPFSettings, @Nullable Boolean staticObstaclesForUnassignedAgents) {

        this.lowLevelSolver = Objects.requireNonNullElseGet(lowLevelSolver, SingleAgentAStar_Solver::new);
        this.openList = Objects.requireNonNullElseGet(openList, OpenListHeap::new);
        this.openListManagementMode = openListManagementMode != null ? openListManagementMode : OpenListManagementMode.AUTOMATIC;
        this.corridorReasoning = Objects.requireNonNullElse(useCorridorReasoning, false);
        clearOPEN();
        // if a specific cost function is not provided, use standard SOC (Sum of Individual Costs)
        this.costFunction = Objects.requireNonNullElseGet(costFunction, SumOfCosts::new);
        this.CBSNodeComparator = cbsNodeComparator != null ? cbsNodeComparator : new CBSNodeComparatorForcedTotalOrdering();
        this.sharedGoals = Objects.requireNonNullElse(sharedGoals, false);
        this.sharedSources = Objects.requireNonNullElse(sharedSources, false);
        this.transientMAPFSettings = Objects.requireNonNullElse(transientMAPFSettings, TransientMAPFSettings.defaultRegularMAPF);
        if (Config.WARNING >= 1 && this.sharedGoals && this.transientMAPFSettings.isTransientMAPF()){
            System.err.println("Warning: " + this.name + " has shared goals and is set to transient MAPF. Shared goals is unnecessary if transient.");
        }

        this.staticObstaclesForUnassignedAgents = Objects.requireNonNullElse(staticObstaclesForUnassignedAgents, false);

        if (this.transientMAPFSettings.isTransientMAPF()) {
            super.name = "CBS_Dynamic_UA";
        } else if (this.staticObstaclesForUnassignedAgents) {
            super.name = "CBS_Static_UA";
        } else {
            super.name = "CBS";
        }
    }

    /**
     * Default constructor.
     */
    CBS_Solver() {
        this(null, null, null, null, null, null, null, null, null, null);
    }

    /*  = initialization =  */

    @Override
    protected void init(MAPF_Instance instance, RunParameters runParameters) {
        super.init(instance, runParameters);

        this.initialConstraints = Objects.requireNonNullElseGet(runParameters.constraints, ConstraintSet::new);
        this.initialSolution = runParameters.existingSolution;

        // All UA is static obstacles
        if (this.staticObstaclesForUnassignedAgents) {
            this.unassignedAgents = new ArrayList<>();
            List<Agent> newAgentsList = new ArrayList<>();
            for (Agent agent : instance.agents) {
                if (agent.isUnassigned) {
                    Constraint infiniteConstraint = new GoalConstraint(null, 1, null, instance.map.getMapLocation(agent.source), agent);
                    this.initialConstraints.add(infiniteConstraint);
                    this.unassignedAgents.add(agent);
                }
                else {
                    newAgentsList.add(agent);
                }
            }
            instance = new MAPF_Instance(instance.name, instance.map, newAgentsList.toArray(new Agent[0]));
        }

        // Some of UA is static obstacles
        if (this.staticUA != null && !this.staticUA.isEmpty()) {
            this.unassignedAgents = new ArrayList<>();
            List<Agent> newAgentsList = new ArrayList<>();
            for (Agent agent : instance.agents) {
                if (staticUA.contains(agent)) {
                    // add infinite constraint to make it static
                    Constraint infiniteConstraint = new GoalConstraint(null, 1, null, instance.map.getMapLocation(agent.source), agent);
                    this.initialConstraints.add(infiniteConstraint);
                    this.unassignedAgents.add(agent);
                } else {
                    newAgentsList.add(agent); // transient agents
                }
            }
            instance = new MAPF_Instance(instance.name, instance.map, newAgentsList.toArray(new Agent[0]));

            // DEBUG: only after unassignedAgents and newAgentsList were built
            System.out.println("[CBS DEBUG] staticUA filtering applied");

            System.out.println("[CBS DEBUG] static UA:");
            for (Agent a : this.unassignedAgents) {
                System.out.println("  STATIC id=" + a.iD
                        + (a.isUnassigned ? " [UA]" : " [AA]")
                        + " " + a.source + "->" + a.target);
            }

            System.out.println("[CBS DEBUG] dynamic agents kept in CBS:");
            for (Agent a : instance.agents) {
                System.out.println("  DYNAMIC id=" + a.iD
                        + (a.isUnassigned ? " [UA]" : " [AA]")
                        + " " + a.source + "->" + a.target);
            }
        }

        this.currentConstraints = new ConstraintSet();
        this.generatedNodes = 0;
        this.expandedNodes = 0;
        this.instance = instance;
        this.seenConstraintSignatures.clear();
        this.forcedMovedUAIds.clear();

        // heuristic
        if (runParameters.singleAgentGAndH != null){
            this.singleAgentGAndH = runParameters.singleAgentGAndH;
        }
        else {
            if (this.lowLevelSolver instanceof SingleAgentAStar_Solver){
                this.singleAgentGAndH = new DistanceTableSingleAgentHeuristic(new ArrayList<>(instance.agents), instance.map);
            }

            if (this.singleAgentGAndH instanceof CachingDistanceTableHeuristic){
                ((CachingDistanceTableHeuristic)this.singleAgentGAndH).setCurrentMap(instance.map);
            }

            if (this.singleAgentGAndH != null) {
                if (this.costFunction instanceof SumServiceTimes) {
                    this.singleAgentGAndH = new ServiceTimeGAndH(this.singleAgentGAndH);
                } else if (this.costFunction instanceof SumFuel) {
                    this.singleAgentGAndH = new FuelGAndH(this.singleAgentGAndH);
                } else if (this.costFunction instanceof SumNUA) {
                    this.singleAgentGAndH = new NUAGAndH(this.singleAgentGAndH);
                }
            }
        }

        if (this.transientMAPFSettings.avoidSeparatingVertices()) {
            this.separatingVerticesSet = TransientMAPFUtils.createSeparatingVerticesSetOfCoordinates(instance, runParameters);
        }
    }

    /*  = algorithm =  */

    /**
     * Implements the CBS algorithm, as described in the original CBS article from Proceedings of the Twenty-Sixth AAAI
     * Conference on Artificial Intelligence.
     * @param instance {@inheritDoc}
     * @param parameters {@inheritDoc}
     * @return {@inheritDoc}
     */
    @Override
    protected Solution runAlgorithm(MAPF_Instance instance, RunParameters parameters) {
        initOpen(Objects.requireNonNullElseGet(this.initialConstraints, ConstraintSet::new));
        CBS_Node goal = mainLoop();
        return solutionFromGoal(goal);
    }

    /**
     * Initialises the {@link #openList OPEN} and inserts the root.
     * @param initialConstraints a set of initial constraints on the agents.
     */
    private void initOpen(I_ConstraintSet initialConstraints) {
        if(this.openListManagementMode == OpenListManagementMode.AUTOMATIC ||
                this.openListManagementMode == OpenListManagementMode.AUTO_INIT_MANUAL_CLEAR){
            addToOpen(generateRoot(initialConstraints));
        }
    }

    /**
     * Creates a root node.
     */

    private CBS_Node generateRoot(I_ConstraintSet initialConstraints) {
        // if an initial solution was provided, use it (may be partial)
        Solution solution;
        if (this.initialSolution != null) {
            solution = this.transientMAPFSettings.isTransientMAPF() ? new TransientMAPFSolution(this.initialSolution) : new Solution(this.initialSolution);
            // ensure all agents have a plan; replan missing ones
            for (Agent agent : this.instance.agents) {
                if (solution.getPlanFor(agent) == null) {
                    solution = solveSubproblem(agent, solution, initialConstraints);
                    if (solution == null) {
                        return null;
                    }
                }
            }
        } else {
            solution = this.transientMAPFSettings.isTransientMAPF() ? new TransientMAPFSolution() : new Solution();
            // for every agent, add its plan to the solution
            for (Agent agent : this.instance.agents) {
                solution = solveSubproblem(agent, solution, initialConstraints);
                if (solution == null){
                    // failed to solve for some agent
                    return null;
                }
            }
        }

        return new CBS_Node(solution, computeNodeCost(solution));
    }

    /**
     * The main loop of the CBS algorithm. Expands and generates nodes.
     * @return the goal node, or null if a timeout occurs before it is found.
     */
    private CBS_Node mainLoop() {
        while(!openList.isEmpty() && !checkTimeout()){
            CBS_Node node = openList.poll();

            if (this.costFunction instanceof SumNUA) {
                float updatedCost = computeNodeCost(node.getSolution());

                if (updatedCost > node.solutionCost) {
                    node.solutionCost = updatedCost;
                    openList.add(node);
                    continue;
                }
            }

            // Debugging printing for NUA-CBS
            if (this.costFunction instanceof SumNUA) {
                System.out.println("\n[NUA-CBS] Poll node"
                        + " id=" + node.serialID
                        + " cost=" + node.getSolutionCost()
                        + " NUA=" + node.getSolution().NUA()
                        + " endTime=" + node.getSolution().endTime()
                        + " openSize=" + openList.size());
            }

            // verify solution (find conflicts)
            I_ConflictManager cat = getConflictManagerFor(node);
            node.setSelectedConflict(cat.selectConflict());

            // If there are no conflicts, this is a valid solution.
            // Do NOT apply TRDP before accepting a conflict-free solution.
            if (isGoal(node)) {
                if (this.costFunction instanceof SumNUA) {
                    System.out.println("[NUA-CBS] Goal candidate: no conflicts");
                }
                return node;
            }

            // NUA-CBS + TRDP:
            // If the current solution is not a goal and contains a repeated global
            // configuration, branch on the TRD conflict instead of a normal pairwise conflict.
            if (this.costFunction instanceof SumNUA) {
                TRDConflict trdConflict = findTRDConflict(node.getSolution());

            if (trdConflict != null && node.getSolution().NUA() == 0) {
                System.out.println("[NUA-CBS-TRDP] Pruning NUA=0 node with repeated global configuration"
                        + " from t=" + trdConflict.loopStart
                        + " to t=" + trdConflict.loopEnd
                        + " | length=" + trdConflict.loopLength);

                continue;
            }
            }

            if (this.costFunction instanceof SumNUA) {
                System.out.println("[NUA-CBS] Selected conflict: " + node.selectedConflict);
            }

            if (this.transientMAPFSettings.resolveAfterGoalConflictsLocally()) {
                if (tryResolveConflictsLocally(node)) {
                    openList.add(node);
                    continue;
                }
            }

            expandNode(node);
        }

        return null; //probably a timeout
    }

    public void setStaticUA(Set<Agent> staticUA) {
        this.staticUA = staticUA;
    }

    /**
     * When a node is first generated, it is given the same {@link ConflictManager} as its parent. Only when that
     * node is later dequeued from {@link #openList}, will we update the table.
     * @param node a {@link CBS_Node node} that contains an out of date {@link ConflictManager}.
     * @return a {@link I_ConflictManager} for the solution in this node.
     */
    private I_ConflictManager getConflictManagerFor(CBS_Node node) {
        I_ConflictManager cat = this.corridorReasoning ?
                new CorridorConflictManager(buildConstraintSet(node,null), this.instance) :
                new ConflictManager(null, this.sharedGoals, this.sharedSources);
        for (SingleAgentPlan plan :
                node.getSolution()) {
            cat.addPlan(plan);
        }
        return cat;
    }

    private boolean isGoal(CBS_Node node) {
        // no conflicts -> found goal
        return node.selectedConflict == null;
    }

    /**
     * Expands a {@link CBS_Node}.
     * @param node a node to expand.
     */

    private void expandNode(CBS_Node node) {

        // Debugging printing for NUA-CBS
        if (this.costFunction instanceof SumNUA) {
            System.out.println("[NUA-CBS] Expanding node"
                    + " id=" + node.serialID
                    + " cost=" + node.getSolutionCost()
                    + " conflict=" + node.selectedConflict);
        }

        this.expandedNodes++;

        // Special handling for FUEL: if there is a timestep where all agents stay,
        // branch into one child per agent, forbidding that agent to stay at that timestep.
        if (this.costFunction instanceof SumFuel) {
        // if (this.costFunction instanceof SumFuel || this.costFunction instanceof SumNUA) {
            Solution sol = node.getSolution();
            int last = sol.endTime();
            for (int t = 1; t <= last; t++) {
                boolean allStayed = true;
                for (SingleAgentPlan plan : sol) {
                    if (!plan.StayInIndex(t)) {
                        allStayed = false;
                        break;
                    }
                }
                if (allStayed) {
                    // create a child per agent that forbids staying at time t at its location
                    List<CBS_Node> children = new ArrayList<>();
                    for (SingleAgentPlan plan : sol) {
                        Agent a = plan.agent;
                        // get agent location at time t (handles before/after plan bounds)
                        BasicMAPF.Instances.Maps.I_Location loc = sol.getAgentLocation(a, t);
                        Constraint cons = new Constraint(a, t, loc, loc); // forbids stay (prevLocation == location)
                        CBS_Node child = generateNode(node, cons, true); // make copy for safety
                        if (child != null) {
                            children.add(child);
                            addToOpen(child);
                        }
                    }
                    // attach first two children to left/right if present (keeps existing structure)
                    if (children.size() > 0) node.leftChild = children.get(0);
                    if (children.size() > 1) node.rightChild = children.get(1);
                    return;
                }
            }
        }

        // default (pairwise conflict) expansion
        Constraint[] constraints = node.selectedConflict.getPreventingConstraints();

        // Debugging printing for NUA-CBS
        if (this.costFunction instanceof SumNUA) {
            System.out.println("[NUA-CBS] Generating CBS children");

            Agent a1 = constraints[0].agent;
            Agent a2 = constraints[1].agent;

            System.out.println("[NUA-CBS] Conflict agents:"
                    + " c0.agent=" + a1.iD + (a1.isUnassigned ? " [UA]" : " [AA]")
                    + ", c1.agent=" + a2.iD + (a2.isUnassigned ? " [UA]" : " [AA]"));

            SingleAgentPlan p1 = node.getSolution().getPlanFor(a1);
            SingleAgentPlan p2 = node.getSolution().getPlanFor(a2);

            System.out.println("[NUA-CBS] p1.nua=" + (a1.isUnassigned ? p1.nua() : -1));
            System.out.println("[NUA-CBS] p2.nua=" + (a2.isUnassigned ? p2.nua() : -1));

            System.out.println("[NUA-CBS] Left constraint:  " + constraints[0]);
            System.out.println("[NUA-CBS] Right constraint: " + constraints[1]);

            if (tryExpandNUAConflictWithStaticUABranch(node, constraints)) {
                return;
            }

            if (tryExpandNUAConflictWithActiveUAAtAASourceOrTarget(node, constraints)) {
                return;
            }
        }

        node.leftChild = generateNode(node, constraints[0], true);
        node.rightChild = generateNode(node, constraints[1], false);

        addToOpen(node.leftChild);
        addToOpen(node.rightChild);
    }

    /**
     * Special branching for NUA-only CBS.
     *
     * Handles conflicts between:
     *   - any moving / active agent
     *   - a stationary UA that has not moved yet
     *
     * The active agent can be:
     *   - an assigned agent
     *   - or a UA that already moved
     *
     * Standard CBS creates a one-time constraint:
     *   activeAgent cannot be at stationaryUA.source at time t
     *
     * In NUA, this is too weak.
     * If the stationary UA does not move, its source is blocked forever.
     *
     * Therefore we create:
     *
     *   Child 1:
     *      active agent avoids the stationary UA location forever.
     *
     *   Child 2:
     *      stationary UA receives the regular conflict constraint and may move.
     *
     * @return true if the special NUA branching was applied.
     */
    private boolean tryExpandNUAConflictWithStaticUABranch(CBS_Node node, Constraint[] constraints) {
        if (!(this.costFunction instanceof SumNUA)) {
            return false;
        }

        if (constraints == null || constraints.length < 2) {
            return false;
        }

        Constraint c0 = constraints[0];
        Constraint c1 = constraints[1];

        Agent agent0 = c0.agent;
        Agent agent1 = c1.agent;

        if (agent0 == null || agent1 == null) {
            return false;
        }

        SingleAgentPlan plan0 = node.getSolution().getPlanFor(agent0);
        SingleAgentPlan plan1 = node.getSolution().getPlanFor(agent1);

        if (plan0 == null || plan1 == null) {
            return false;
        }

        boolean agent0IsStationaryUA = agent0.isUnassigned && plan0.nua() == 0;
        boolean agent1IsStationaryUA = agent1.isUnassigned && plan1.nua() == 0;

        /*
        * We need exactly one stationary UA.
        *
        * If none are stationary UA -> regular CBS.
        * If both are stationary UA -> usually invalid/rare, and regular CBS is safer.
        */
        if (agent0IsStationaryUA == agent1IsStationaryUA) {
            return false;
        }

        Agent stationaryUA;
        Agent activeAgent;
        Constraint stationaryUARegularConstraint;

        if (agent0IsStationaryUA) {
            stationaryUA = agent0;
            activeAgent = agent1;
            stationaryUARegularConstraint = c0;
        } else {
            stationaryUA = agent1;
            activeAgent = agent0;
            stationaryUARegularConstraint = c1;
        }

        I_Location stationaryLocation = this.instance.map.getMapLocation(stationaryUA.source);

        if (!activeAgent.isUnassigned
            && !canAgentReachTargetWithoutLocation(activeAgent, stationaryLocation)) {
            if (forcedMovedUAIds.add(stationaryUA.iD)) {
                System.out.println("[NUA-CBS] Forced UA detected: UA "
                        + stationaryUA.iD
                        + " must move because AA "
                        + activeAgent.iD
                        + " cannot reach its target while "
                        + stationaryLocation
                        + " is blocked");
            }
        }

        System.out.println("[NUA-CBS] Applying special active-agent vs stationary-UA branch");
        System.out.println("[NUA-CBS] Child 1: active agent " + activeAgent.iD
                + (activeAgent.isUnassigned ? " [UA]" : " [AA]")
                + " avoids stationary UA " + stationaryUA.iD
                + " at " + stationaryLocation + " forever");
        System.out.println("[NUA-CBS] Child 2: stationary UA " + stationaryUA.iD
                + " replans with regular constraint " + stationaryUARegularConstraint);

        /*
        * Child 1:
        * Keep the stationary UA static.
        * Therefore the active agent must avoid the stationary UA source forever.
        */
        Constraint activeAvoidsStationaryUAForever = new GoalConstraint(
                activeAgent,
                1,
                null,
                stationaryLocation,
                stationaryUA
        );

        CBS_Node activeAvoidChild = generateNode(node, activeAvoidsStationaryUAForever, true);

        /*
        * Child 2:
        * Force the stationary UA to resolve the conflict.
        * This may increase NUA by 1.
        */
        CBS_Node stationaryUAMovesChild = generateNode(node, stationaryUARegularConstraint, true);

        node.leftChild = activeAvoidChild;
        node.rightChild = stationaryUAMovesChild;

        addToOpen(node.leftChild);
        addToOpen(node.rightChild);

        return true;
    }

    private boolean tryExpandNUAConflictWithActiveUAAtAASourceOrTarget(CBS_Node node, Constraint[] constraints) {
        if (!(this.costFunction instanceof SumNUA)) {
            return false;
        }

        if (constraints == null || constraints.length < 2) {
            return false;
        }

        Constraint c0 = constraints[0];
        Constraint c1 = constraints[1];

        Agent a0 = c0.agent;
        Agent a1 = c1.agent;

        if (a0 == null || a1 == null) {
            return false;
        }

        if (a0.isUnassigned == a1.isUnassigned) {
            return false;
        }

        Agent aaAgent = a0.isUnassigned ? a1 : a0;
        Agent uaAgent = a0.isUnassigned ? a0 : a1;

        SingleAgentPlan uaPlan = node.getSolution().getPlanFor(uaAgent);
        if (uaPlan == null || uaPlan.nua() == 0) {
            return false;
        }

        I_Location aaSource = this.instance.map.getMapLocation(aaAgent.source);
        I_Location aaTarget = this.instance.map.getMapLocation(aaAgent.target);

        I_Location conflictLocation = c0.location;

        if (conflictLocation == null) {
            return false;
        }

        boolean conflictAtAASourceOrTarget =
                conflictLocation.equals(aaSource) || conflictLocation.equals(aaTarget);

        if (!conflictAtAASourceOrTarget) {
            return false;
        }

        System.out.println("[NUA-CBS] Active UA blocks AA source/target");
        System.out.println("[NUA-CBS] Forcing UA " + uaAgent.iD
                + " to avoid " + conflictLocation + " forever");

        Constraint uaAvoidsAASourceOrTargetForever = new GoalConstraint(
                uaAgent,
                1,
                null,
                conflictLocation,
                aaAgent
        );

        CBS_Node child = generateNode(node, uaAvoidsAASourceOrTargetForever, true);

        node.leftChild = child;
        node.rightChild = null;

        addToOpen(node.leftChild);

        return true;
    }

    private boolean tryExpandAADeadlockByActivatingUA(CBS_Node node, Constraint[] constraints) {
        if (!(this.costFunction instanceof SumNUA)) {
            return false;
        }

        if (constraints == null || constraints.length < 2) {
            return false;
        }

        Agent a0 = constraints[0].agent;
        Agent a1 = constraints[1].agent;

        if (a0 == null || a1 == null) {
            return false;
        }

        // This reasoning is only for AA-AA conflicts.
        if (a0.isUnassigned || a1.isUnassigned) {
            return false;
        }

        /*
        * Important:
        * Only UAs that have NOT moved yet are still blocking their source as static buffers.
        * If a UA already moved, do not keep treating its source as blocked here.
        */
        Set<Agent> blockingUAs = getStationaryUAs(node);

        if (blockingUAs.isEmpty()) {
            return false;
        }

        Set<I_Location> blockedByUASources = getSourceLocations(blockingUAs);

        /*
        * Can these two AAs solve their pairwise problem while the currently-stationary
        * UA source cells are blocked?
        */
        boolean feasibleWithCurrentBlockedUASources =
                twoAgentsFeasibleWithBlockedLocations(a0, a1, blockedByUASources);

        if (feasibleWithCurrentBlockedUASources) {
            return false;
        }

        System.out.println("[NUA-CBS] AA-AA deadlock detected between AA "
                + a0.iD + " and AA " + a1.iD
                + " while currently-stationary UA source cells are blocked");

        List<Agent> activationCandidates = new ArrayList<>();

        /*
        * Try to find which stationary UA source opens the deadlock.
        */
        for (Agent ua : blockingUAs) {
            I_Location uaSource = this.instance.map.getMapLocation(ua.source);

            if (uaSource == null) {
                continue;
            }

            Set<I_Location> blockedWithoutThisUA = new HashSet<>(blockedByUASources);
            blockedWithoutThisUA.remove(uaSource);

            boolean feasibleIfThisSourceIsFree =
                    twoAgentsFeasibleWithBlockedLocations(a0, a1, blockedWithoutThisUA);

            if (feasibleIfThisSourceIsFree) {
                activationCandidates.add(ua);
            }
        }

        if (activationCandidates.isEmpty()) {
            return false;
        }

        if (activationCandidates.size() == 1) {
            Agent forcedUA = activationCandidates.get(0);
            forcedMovedUAIds.add(forcedUA.iD);

            System.out.println("[NUA-CBS] Forced UA detected from AA-AA deadlock: UA "
                    + forcedUA.iD + " must move / free its source");
        }

        boolean generatedAnyChild = false;

        for (Agent ua : activationCandidates) {
            I_Location uaSource = this.instance.map.getMapLocation(ua.source);

            if (uaSource == null) {
                continue;
            }

            System.out.println("[NUA-CBS] Activating UA " + ua.iD
                    + " and forcing it to avoid its source forever: "
                    + uaSource);

            Constraint keepUASourceFreeForever = new GoalConstraint(
                    ua,
                    1,
                    null,
                    uaSource,
                    a0
            );

            CBS_Node child = generateNode(node, keepUASourceFreeForever, true);

            if (addToOpen(child)) {
                generatedAnyChild = true;
            }
        }

        return generatedAnyChild;
    }

    private Set<Agent> getStationaryUAs(CBS_Node node) {
        Set<Agent> stationary = new HashSet<>();

        for (SingleAgentPlan plan : node.getSolution()) {
            Agent agent = plan.agent;

            if (agent.isUnassigned && plan.nua() == 0) {
                stationary.add(agent);
            }
        }

        return stationary;
    }

    private Set<Agent> getAllUAsInCurrentInstance() {
        Set<Agent> result = new HashSet<>();

        for (Agent agent : this.instance.agents) {
            if (agent.isUnassigned) {
                result.add(agent);
            }
        }

        return result;
    }

    private Set<I_Location> getSourceLocations(Set<Agent> agents) {
        Set<I_Location> locations = new HashSet<>();

        for (Agent agent : agents) {
            I_Location location = this.instance.map.getMapLocation(agent.source);

            if (location != null) {
                locations.add(location);
            }
        }

        return locations;
    }

    private boolean twoAgentsFeasibleWithBlockedLocations(
            Agent agentA,
            Agent agentB,
            Set<I_Location> blockedLocations
    ) {
        I_Location startA = this.instance.map.getMapLocation(agentA.source);
        I_Location targetA = this.instance.map.getMapLocation(agentA.target);

        I_Location startB = this.instance.map.getMapLocation(agentB.source);
        I_Location targetB = this.instance.map.getMapLocation(agentB.target);

        if (startA == null || targetA == null || startB == null || targetB == null) {
            return false;
        }

        if (blockedLocations.contains(startA)
                || blockedLocations.contains(targetA)
                || blockedLocations.contains(startB)
                || blockedLocations.contains(targetB)) {
            return false;
        }

        LocationPair start = new LocationPair(startA, startB);

        Queue<LocationPair> queue = new ArrayDeque<>();
        Set<LocationPair> visited = new HashSet<>();

        queue.add(start);
        visited.add(start);

        while (!queue.isEmpty()) {
            LocationPair curr = queue.poll();

            if (curr.locationA.equals(targetA) && curr.locationB.equals(targetB)) {
                return true;
            }

            List<I_Location> nextLocationsA = getLegalNextLocations(curr.locationA, blockedLocations);
            List<I_Location> nextLocationsB = getLegalNextLocations(curr.locationB, blockedLocations);

            for (I_Location nextA : nextLocationsA) {
                for (I_Location nextB : nextLocationsB) {

                    // Vertex conflict.
                    if (nextA.equals(nextB)) {
                        continue;
                    }

                    // Swapping conflict.
                    if (nextA.equals(curr.locationB) && nextB.equals(curr.locationA)) {
                        continue;
                    }

                    LocationPair nextPair = new LocationPair(nextA, nextB);

                    if (visited.add(nextPair)) {
                        queue.add(nextPair);
                    }
                }
            }
        }

        return false;
    }

    private List<I_Location> getLegalNextLocations(I_Location current, Set<I_Location> blockedLocations){
        List<I_Location> result = new ArrayList<>();

        // Wait action.
        if (!blockedLocations.contains(current)) {
            result.add(current);
        }

        for (I_Location next : current.outgoingEdges()) {
            if (!blockedLocations.contains(next)) {
                result.add(next);
            }
        }

        return result;
    }

    private TRDConflict findTRDConflict(Solution solution) {
        if (solution == null) {
            return null;
        }

        Map<String, Integer> firstSeenTimeByConfiguration = new HashMap<>();

        int endTime = solution.endTime();

        for (int t = 0; t <= endTime; t++) {
            String key = globalConfigurationKey(solution, t);

            if (key == null) {
                continue;
            }

            Integer firstTime = firstSeenTimeByConfiguration.get(key);

            if (firstTime != null) {
                if (t > firstTime) {
                    return new TRDConflict(firstTime, t);
                }
            } else {
                firstSeenTimeByConfiguration.put(key, t);
            }
        }

        return null;
    }

    private String globalConfigurationKey(Solution solution, int time) {
        List<String> parts = new ArrayList<>();

        for (SingleAgentPlan plan : solution) {
            Agent agent = plan.agent;

            I_Location location = solution.getAgentLocation(agent, time);

            if (location == null || location.getCoordinate() == null) {
                return null;
            }

            parts.add(agent.iD + "@" + location.getCoordinate().toString());
        }

        parts.sort(String::compareTo);

        return String.join("|", parts);
    }

    private void expandTRDNode(CBS_Node node, TRDConflict trdConflict) {
        this.expandedNodes++;

        List<CBS_Node> children = new ArrayList<>();

        Solution solution = node.getSolution();

        for (SingleAgentPlan plan : solution) {
            Agent agent = plan.agent;

            I_Location repeatedLocation = solution.getAgentLocation(agent, trdConflict.loopEnd);

            if (repeatedLocation == null) {
                continue;
            }

            /*
            * Practical TRDP branch:
            * The global configuration at loopEnd duplicates the one at loopStart.
            * Therefore, at least one agent must not repeat its loop-end position.
            *
            * This is the classic CBS idea generalized:
            * instead of branching on two agents in a conflict, we branch on all agents
            * involved in the repeated global configuration.
            */
            Constraint breakLoopConstraint = new Constraint(
                    agent,
                    trdConflict.loopEnd,
                    null,
                    repeatedLocation
            );

            System.out.println("[NUA-CBS-TRDP] Child: agent "
                    + agent.iD
                    + (agent.isUnassigned ? " [UA]" : " [AA]")
                    + " cannot be at "
                    + repeatedLocation
                    + " at t=" + trdConflict.loopEnd);

            CBS_Node child = generateNode(node, breakLoopConstraint, true);

            if (addToOpen(child)) {
                children.add(child);
            }
        }

        if (!children.isEmpty()) {
            node.leftChild = children.get(0);
        }

        if (children.size() > 1) {
            node.rightChild = children.get(1);
        }
    }

    /**
     * Adds a node to {@link #openList OPEN}. If a duplicate node exists, keeps the one with less cost.
     * @param node a node to insert into {@link #openList OPEN}
     * @return true if {@link #openList OPEN} changed as a result of the call.
     */
    private boolean addToOpen(CBS_Node node) {
        if(node == null){
            // either the low level encountered a timeout, or the low level was unsolvable.
            return false;
        }

        if (this.costFunction instanceof SumNUA) {
            String signature = constraintPathSignature(node);

            if (seenConstraintSignatures.contains(signature)) {
                System.out.println("[NUA-CBS] Duplicate node pruned");
                return false;
            }

            seenConstraintSignatures.add(signature);
        }

        return openList.add(node);
    }

    private float computeNodeCost(Solution solution) {
        if (!(this.costFunction instanceof SumNUA)) {
            return this.costFunction.solutionCost(solution);
        }

        int cost = solution.NUA();

        for (Integer forcedUAId : forcedMovedUAIds) {
            if (!isUAIdMovedInSolution(solution, forcedUAId)) {
                cost++;
            }
        }

        return cost;
    }

    private boolean isUAIdMovedInSolution(Solution solution, int uaId) {
        for (SingleAgentPlan plan : solution) {
            if (plan.agent.iD == uaId) {
                return plan.agent.isUnassigned && plan.nua() != 0;
            }
        }

        return false;
    }

    private boolean canAgentReachTargetWithoutLocation(Agent agent, I_Location blockedLocation) {
        I_Location source = this.instance.map.getMapLocation(agent.source);
        I_Location target = this.instance.map.getMapLocation(agent.target);

        if (source == null || target == null || blockedLocation == null) {
            return false;
        }

        if (source.equals(blockedLocation) || target.equals(blockedLocation)) {
            return false;
        }

        Queue<I_Location> queue = new ArrayDeque<>();
        Set<I_Location> visited = new HashSet<>();

        queue.add(source);
        visited.add(source);

        while (!queue.isEmpty()) {
            I_Location curr = queue.poll();

            if (curr.equals(target)) {
                return true;
            }

            for (I_Location next : curr.outgoingEdges()) {
                if (next.equals(blockedLocation)) {
                    continue;
                }

                if (visited.add(next)) {
                    queue.add(next);
                }
            }
        }

        return false;
    }

    private String constraintPathSignature(CBS_Node node) {
        List<String> parts = new ArrayList<>();

        CBS_Node curr = node;
        while (curr != null && curr.addedConstraint != null) {
            parts.add(constraintSignature(curr.addedConstraint));
            curr = curr.parent;
        }

        parts.sort(String::compareTo);
        return String.join(";", parts);
    }

    private String constraintSignature(Constraint c) {
        String agentId = c.agent == null ? "ALL" : String.valueOf(c.agent.iD);
        String prev = c.prevLocation == null ? "null" : c.prevLocation.getCoordinate().toString();
        String loc = c.location == null ? "null" : c.location.getCoordinate().toString();
        String type = c instanceof GoalConstraint ? "GOAL" : "REG";

        return type + "|a=" + agentId + "|t=" + c.time + "|prev=" + prev + "|loc=" + loc;
    }

    /**
     * Since the creation of a new {@link CBS_Node node} is somewhat complicated, it is handled in its own method.
     * @param parent the new node's parent
     * @param constraint the constraint that we want to add in this node, before re-solving the agent that is constrained.
     * @param copyDatastructures for one child, we may be able to reuse the parent's data structures, instead of copying them.
     * @return a new {@link CBS_Node}.
     */
    private CBS_Node generateNode(CBS_Node parent, Constraint constraint, boolean copyDatastructures) {
        Agent agent = constraint.agent;

        // Debugging printing for NUA-CBS
        if (this.costFunction instanceof SumNUA) {
            System.out.println("[NUA-CBS] Generate child for agent "
                    + agent.iD
                    + (agent.isUnassigned ? " [UA]" : " [AA]")
                    + " with constraint: " + constraint);
        }

        Solution solution = parent.solution;

        // replace with copies if required
        if(copyDatastructures) {
            solution =  transientMAPFSettings.isTransientMAPF() ? new TransientMAPFSolution(solution) : new Solution(solution);
        }

        // modify for this node
        /*  replace the current plan for the agent with an empty plan, so that the low level won't try to continue the
            existing plan.
            Also we don't want to reuse (modify) SingleAgentPlan objects, as they are pointed to by other Solution objects, which
            we don't want to modify.
         */
        solution.putPlan(new SingleAgentPlan(agent));


        // Build the full constraint set for this child once.
        I_ConstraintSet nodeConstraints = buildConstraintSet(parent, constraint);

        // The low-level should update the solution.
        Solution agentSolution = solveSubproblem(agent, solution, nodeConstraints);

        if(agentSolution == null) {
            if (this.costFunction instanceof SumNUA) {
                System.out.println("[NUA-CBS] Child pruned: low-level failed for agent "
                        + agent.iD
                        + (agent.isUnassigned ? " [UA]" : " [AA]"));
            }
            return null;
        }

        SingleAgentPlan newPlan = agentSolution.getPlanFor(agent);

        if (newPlan == null) {
            if (this.costFunction instanceof SumNUA) {
                System.out.println("[NUA-CBS] Child pruned: low-level returned no plan for agent "
                        + agent.iD
                        + (agent.isUnassigned ? " [UA]" : " [AA]"));
            }
            return null;
        }

        /*
        * Important:
        * If the new plan itself violates the constraint that generated this child,
        * this child is useless and will reproduce the same conflict.
        */
        if (constraint.rejects(newPlan)) {
            if (this.costFunction instanceof SumNUA) {
                System.out.println("[NUA-CBS] Child pruned: new plan violates its own constraint "
                        + constraint);
            }
            return null;
        }

        /*
        * Important:
        * If the plan ends at a location that will be rejected by some future constraint,
        * then CBS will keep seeing the same conflict after the plan end.
        *
        * This is exactly what happened when UA returned endTime=2 while the conflict was at t=37.
        */
        Move lastMove = newPlan.moveAt(newPlan.getEndTime());

        if (lastMove == null || !nodeConstraints.acceptsForever(lastMove)) {
            if (this.costFunction instanceof SumNUA) {
                System.out.println("[NUA-CBS] Child pruned: plan cannot safely stay forever after endTime"
                        + " agent=" + agent.iD
                        + (agent.isUnassigned ? " [UA]" : " [AA]")
                        + " endTime=" + newPlan.getEndTime());
            }
            return null;
        }

        // Preserve other agents' plans and add the re-planned agent's new plan.
        solution.putPlan(newPlan);

        // Debugging printing for NUA-CBS
        if (this.costFunction instanceof SumNUA) {
            System.out.println("[NUA-CBS] New path for agent "
                    + agent.iD
                    + (agent.isUnassigned ? " [UA]" : " [AA]")
                    + ": size=" + newPlan.size()
                    + ", endTime=" + newPlan.getEndTime()
                    + ", nua=" + (agent.isUnassigned ? newPlan.nua() : -1));
        }

        return new CBS_Node(solution, computeNodeCost(solution), constraint, parent);
    }

    /**
     * When solving a new node, you want a set of constraints that apply to it. To save on memory, this set is created
     * on the spot, by climbing up the CT and collecting all the constraints that were added
     * @param parentNode the new node's parent.
     * @param newConstraint the constraint that this new node adds.
     * @return a {@link ConstraintSet} of all the constraints from parentNode to the root, plus newConstraint.
     */
    private I_ConstraintSet buildConstraintSet(CBS_Node parentNode, Constraint newConstraint) {
        // clear currentConstraints. we reuse this object every time.
        this.currentConstraints.clear();
        I_ConstraintSet constraintSet = this.currentConstraints;
        // start by adding all the constraints that we were asked to start the solver with (and are therefore not in the CT)
        constraintSet.addAll(initialConstraints);

        CBS_Node currentNode = parentNode;
        while (currentNode.addedConstraint != null){ // will skip the root (it has no constraints)
            constraintSet.add(currentNode.addedConstraint);
            currentNode = currentNode.parent;
        }
        if(newConstraint != null){
            constraintSet.add(newConstraint);
        }
        return constraintSet;
    }

    /**
     * Solves a single agent sub-problem.
     * @param agent
     * @param currentSolution
     * @param constraints
     * @return a solution to a single agent sub-problem. Typically the same object as currentSolution, after being modified.
     */
    private Solution solveSubproblem(Agent agent, Solution currentSolution, I_ConstraintSet constraints) {
        InstanceReport instanceReport = new InstanceReport();
        instanceReport.keepSolutionString = false;
        RunParameters subproblemParameters = getSubproblemParameters(currentSolution, constraints, instanceReport, agent);
        Solution subproblemSolution = this.lowLevelSolver.solve(this.instance.getSubproblemFor(agent), subproblemParameters);
        digestSubproblemReport(instanceReport);
        return subproblemSolution;
    }

    private RunParameters getSubproblemParameters(Solution currentSolution, I_ConstraintSet constraints, InstanceReport instanceReport, Agent agent) {
        // if there was already a timeout while solving a node, we will get a negative time left, which would be
        // interpreted as "use default timeout". In such a case we should instead give the solver 0 time to solve.
        long timeLeftToTimeout = Math.max(super.maximumRuntime - (System.nanoTime()/1000000 - super.startTime), 0);
        RunParameters subproblemParametes = new RunParametersBuilder().setTimeout(timeLeftToTimeout).setConstraints(new UnmodifiableConstraintSet(constraints)).
                setInstanceReport(instanceReport).setExistingSolution(currentSolution).setAStarGAndH(this.singleAgentGAndH).createRP();
        if(this.lowLevelSolver instanceof SingleAgentAStar_Solver){ // upgrades to a better heuristic
            RunParameters_SAAStar astarSubproblemParameters = new RunParameters_SAAStar(subproblemParametes);

            // TMAPF goal condition
            if (transientMAPFSettings.isTransientMAPF()) {
                astarSubproblemParameters.goalCondition = TransientMAPFUtils.createLowLevelGoalConditionForTransientMAPF(transientMAPFSettings, separatingVerticesSet, instance.agents, agent, null);
            }

            I_ConflictAvoidanceTable cat;
            if (this.transientMAPFSettings.isTransientMAPF()){
                // Use RemovableConflictAvoidanceTableWithContestedGoals for TMAPF in CBS, because it correctly counts the number of conflicts after a target is reached.
                if (this.sharedGoals || this.sharedSources){
                    // should be verified earlier, but just in case
                    throw new UnsupportedOperationException("Shared goals and shared sources are not supported in TMAPF");
                }
                cat = new RemovableConflictAvoidanceTableWithContestedGoals(currentSolution, agent);
            }
            else{
                cat = new SingleUseConflictAvoidanceTable(currentSolution, agent);
                ((SingleUseConflictAvoidanceTable)cat).sharedGoals = this.sharedGoals;
                ((SingleUseConflictAvoidanceTable)cat).sharedSources = this.sharedSources;
            }
            astarSubproblemParameters.conflictAvoidanceTable = cat;
            subproblemParametes = astarSubproblemParameters;
        }
        return subproblemParametes;
    }

    /**
     * Extracts a solution from a goal {@link CBS_Node node}.
     * @param goal a {@link CBS_Node} that we consider to be a goal node.
     * @return a solution from a goal {@link CBS_Node node}.
     */
    private Solution solutionFromGoal(CBS_Node goal) {
        if(goal == null){
            return null;
        }
        else{
            if (this.staticObstaclesForUnassignedAgents) {
                for (Agent agent : this.unassignedAgents) {
                    SingleAgentPlan plan = new SingleAgentPlan(agent);

                    Move move = new Move(agent, 1, this.instance.map.getMapLocation(agent.source), this.instance.map.getMapLocation(agent.target));
                    plan.addMove(move);
                    goal.solution.putPlan(plan);
                }
            }
            if (this.staticUA != null && !this.staticUA.isEmpty()) {
                for (Agent agent : this.unassignedAgents) {
                    SingleAgentPlan plan = new SingleAgentPlan(agent);
                    Move move = new Move(agent, 1, this.instance.map.getMapLocation(agent.source), this.instance.map.getMapLocation(agent.target));
                    plan.addMove(move);
                    goal.solution.putPlan(plan);
                }
            }
            // return trimTransientMAPFSolution(goal.solution);
            return goal.solution;
        }
    }

    /**
     * Clears OPEN
     */
    private void clearOPEN() {
        if(this.openListManagementMode == OpenListManagementMode.AUTOMATIC ||
                this.openListManagementMode == OpenListManagementMode.MANUAL_INIT_AUTO_CLEAR){
            openList.clear();
        }
    }

    /**
     * The function checks if the conflict in the CBS node happens after one of the agents reached its target.
     * If so, try to resolve it using the function {@link #resolveConflict(Agent, Agent, CBS_Node)}
     * @param node - CBS node containing a conflict to resolve.
     * @return true if the conflict was resolved, false otherwise.
     */
    private boolean tryResolveConflictsLocally(CBS_Node node) {
        // check if the conflict occurs after one agent reached its target
        Agent agent1 = node.selectedConflict.agent1;
        Agent agent2 = node.selectedConflict.agent2;
        int conflictTime = node.selectedConflict.time;
        int agent1VisitedTargetTime = node.solution.getPlanFor(agent1).firstVisitToTargetTime();
        int agent2VisitedTargetTime = node.solution.getPlanFor(agent2).firstVisitToTargetTime();

        // If the conflict occurs after one of the agents reached its target, try to resolve the conflict locally
        boolean conflictResolved = false;
        if (agent1VisitedTargetTime < conflictTime) {
            conflictResolved = resolveConflict(agent1, agent2, node);
        }
        if (agent2VisitedTargetTime < conflictTime) {
            conflictResolved = resolveConflict(agent2, agent1, node);
        }
        return conflictResolved;
    }


    /**
     * Try to resolve a conflict locally. If succeeded, the solution in the CBS node is replaced with the new solution.
     * @param resolvingAgent is the agent that reached its target, and needs to move to allow the other agent to move.
     * @param otherAgent is the agent that does not change its plan.
     * @param node - CBS node contains the current solution.
     * @return True if the conflict is resolved and successfully replaced in node, false otherwise.
     */
    private boolean resolveConflict(Agent resolvingAgent, Agent otherAgent, CBS_Node node) {
        ConstraintSet allConstraints = new ConstraintSet(this.currentConstraints);
        allConstraints.addAll(allConstraints.allConstraintsForPlan(node.solution.getPlanFor(otherAgent)));
        Solution newSolution = solveSubproblem(resolvingAgent, node.solution, allConstraints);
        if (newSolution == null) {
            return false;
        }
        else {
            node.solution = newSolution;
            return true;
        }
    }


    /*  = wind down =  */

    @Override
    protected void writeMetricsToReport(Solution solution) {
        super.writeMetricsToReport(solution);
        if(solution != null){
            super.instanceReport.putStringValue(InstanceReport.StandardFields.solutionCostFunction, costFunction.name());
            super.instanceReport.putFloatValue(InstanceReport.StandardFields.solutionCost, costFunction.solutionCost(solution));
        }
    }

    @Override
    protected void releaseMemory() {
        clearOPEN();
        this.initialConstraints = null;
        this.currentConstraints = null;
        this.instance = null;
        this.singleAgentGAndH = null;
    }

    /*  = internal classes and interfaces =  */

    /**
     * A data type for representing a single node in the CBS search tree.
     * Try to keep most logic in {@link CBS_Solver}, avoiding methods in this class.
     */
    public class CBS_Node implements Comparable<CBS_Node>{

        /*  =  = fields =  */

        /**
         * The solution in this node. For every non-root node, this solution is after rerouting (solving low level) an
         * agent to overcome a conflict.
         * Holds references to the same {@link SingleAgentPlan plans} as in {@link #parent}, apart from the plan
         * of the re-routed agent.
         */
        private Solution solution;
        /**
         * The cost of the solution.
         */
        private float solutionCost;
        /**
         * The constraint that was added in this node (missing from {@link #parent}).
         */
        private Constraint addedConstraint;
        /**
         * A {@link A_Conflict conflict}, selected to be solved by new constraints in child nodes.
         */
        private A_Conflict selectedConflict;
        /**
         * Needed to enforce total ordering on nodes, which is needed to make node expansions fully deterministic. That
         * is to say, if all tie breaking methods still result in equality, tie break for using serialID.
         */
        private final int serialID = CBS_Solver.this.generatedNodes++; // take and increment

        /*  =  =  = CBS tree branches =  =  */

        /**
         * This node's parent node. This node's {@link #addedConstraint} solves parent's {@link #selectedConflict}.
         */
        private CBS_Node parent;
        /**
         * One of this node's child nodes. Solves this node's {@link #selectedConflict} in one way.
         */
        private CBS_Node leftChild;
        /**
         * One of this node's child nodes. Solves this node's {@link #selectedConflict} in one way.
         */
        private CBS_Node rightChild;

        /*  =  = constructors =  */

        /**
         * Root constructor.
         * @param solution an initial solution for all agents.
         * @param solutionCost the cost of the solution.
         */
        public CBS_Node(Solution solution, float solutionCost) {
            this.solution = solution;
            this.solutionCost = solutionCost;
            this.parent = null;
        }

        /**
         * Non-root constructor.
         */
        public CBS_Node(Solution solution, float solutionCost, Constraint addedConstraint, CBS_Node parent) {
            this.solution = solution;
            this.solutionCost = solutionCost;
            this.addedConstraint = addedConstraint;
            this.parent = parent;
        }

        /*  =  = when expanding a node =  */

        /**
         * Set the selected conflict. Typically done through delegation to {@link I_ConflictManager#selectConflict()}.
         * @param selectedConflict
         */
        public void setSelectedConflict(A_Conflict selectedConflict) {
            this.selectedConflict = selectedConflict;
        }

        /**
         * Set a reference to one of the generated child nodes when expanding this node.
         * @param leftChild One of this node's child nodes. Solves this node's {@link #selectedConflict} in one way.
         */
        public void setLeftChild(CBS_Node leftChild) {
            this.leftChild = leftChild;
        }

        /**
         * Set a reference to one of the generated child nodes when expanding this node.
         * @param rightChild One of this node's child nodes. Solves this node's {@link #selectedConflict} in one way.
         */
        public void setRightChild(CBS_Node rightChild) {
            this.rightChild = rightChild;
        }

        /*  =  = getters =  */

        public Solution getSolution() {
            return solution;
        }

        public float getSolutionCost() {
            return solutionCost;
        }

        public Constraint getAddedConstraint() {
            return addedConstraint;
        }

        public A_Conflict getSelectedConflict() {
            return selectedConflict;
        }

        public CBS_Node getParent() {
            return parent;
        }

        public CBS_Node getLeftChild() {
            return leftChild;
        }

        public CBS_Node getRightChild() {
            return rightChild;
        }

        @Override
        public int compareTo(CBS_Node o) {
            return Objects.compare(this, o, CBS_Solver.this.CBSNodeComparator);
        }

    }

    public static class CBSNodeComparatorForcedTotalOrdering implements Comparator<CBS_Node> {

        @Override
        public int compare(CBS_Node o1, CBS_Node o2) {
            int costCompare = Float.compare(o1.getSolutionCost(), o2.getSolutionCost());
            if (costCompare != 0) {
                return costCompare;
            }

            /*
            * Technical tie-break only.
            * This does not change the optimization objective.
            * It only helps CBS avoid exploring very long equal-NUA branches first.
            */
            int endTimeCompare = Integer.compare(
                    o1.getSolution().endTime(),
                    o2.getSolution().endTime()
            );
            if (endTimeCompare != 0) {
                return endTimeCompare;
            }

            // Final deterministic tie-break.
            return Integer.compare(o1.serialID, o2.serialID);
        }
    }


    /**
     * Modes for handling the initialization and clearing of {@link #openList OPEN}. The default mode of operation is
     * {@link #AUTOMATIC}.
     */
    public enum OpenListManagementMode{
        /**
         * Will handle OPEN automatically. This is the standard mode of operation. The solver will clear OPEN before and
         * after every run, and initialize OPEN at the start of every run with a single root {@link CBS_Node node}.
         */
        AUTOMATIC,
        /**
         * Will initialize OPEN automatically, but clearing it before or after a run will be controlled manually.
         * Note that this means the solver keeps part of its state after running. If you want to reuse the solver, you
         * have to manually handle the clearing of OPEN. If you keep references to many such solvers, this may adversely
         * affect available memory.
         */
        AUTO_INIT_MANUAL_CLEAR,
        /**
         * Will not initialize OPEN (assumes that it was already initialized), but will clear it after running.
         * It is not cleared before running. If it were to be cleared before running, manual initialization would be
         * impossible.
         */
        MANUAL_INIT_AUTO_CLEAR,
        /**
         * Will not initialize OPEN (assumes that it was already initialized).
         * Will not clear OPEN automatically.
         * Note that this means the solver keeps part of its state after running. If you want to reuse the solver, you
         * have to manually handle the clearing of OPEN. If you keep references to many such solvers, this may adversely
         * affect available memory.
         */
        MANUAL
    }

    private static class TRDConflict {
        private final int loopStart;
        private final int loopEnd;
        private final int loopLength;

        private TRDConflict(int loopStart, int loopEnd) {
            this.loopStart = loopStart;
            this.loopEnd = loopEnd;
            this.loopLength = loopEnd - loopStart;
        }
    }

    private static class LocationPair {
        private final I_Location locationA;
        private final I_Location locationB;

        private LocationPair(I_Location locationA, I_Location locationB) {
            this.locationA = locationA;
            this.locationB = locationB;
        }

        @Override
        public boolean equals(Object other) {
            if (this == other) {
                return true;
            }

            if (!(other instanceof LocationPair)) {
                return false;
            }

            LocationPair that = (LocationPair) other;

            return Objects.equals(this.locationA, that.locationA)
                    && Objects.equals(this.locationB, that.locationB);
        }

        @Override
        public int hashCode() {
            return Objects.hash(locationA, locationB);
        }
    }


    /**
     * Trims unnecessary moves from plans after agents reach their goals in Transient MAPF.
     */
    public Solution trimTransientMAPFSolution(Solution solution) {
        if (!this.transientMAPFSettings.isTransientMAPF()) {
            return solution;
        }
        
        Solution trimmedSolution = new TransientMAPFSolution();
        
        for (SingleAgentPlan plan : solution) {
            Agent agent = plan.agent;
            int goalTime = plan.firstVisitToTargetTime();
            
            // If agent never reaches goal, keep full plan
            if (goalTime == -1) {
                trimmedSolution.putPlan(plan);
                continue;
            }
            
            // If plan already ends at or before goal, keep it as-is
            if (plan.getEndTime() <= goalTime) {
                trimmedSolution.putPlan(plan);
                continue;
            }
            
            // Collect moves up to goalTime
            List<Move> trimmedMoves = new ArrayList<>();
            for (int t = plan.getFirstMoveTime(); t <= goalTime; t++) {
                Move move = plan.moveAt(t);
                if (move != null) {
                    trimmedMoves.add(move);
                }
            }
            
            // Create new plan with trimmed moves using constructor
            SingleAgentPlan trimmedPlan = new SingleAgentPlan(agent, trimmedMoves);
            trimmedSolution.putPlan(trimmedPlan);
        }
        
        return trimmedSolution;
    }
}
