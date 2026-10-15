package BasicMAPF.Solvers;

import BasicMAPF.DataTypesAndStructures.Move;
import BasicMAPF.DataTypesAndStructures.RunParametersBuilder;
import BasicMAPF.DataTypesAndStructures.SingleAgentPlan;
import BasicMAPF.DataTypesAndStructures.Solution;
import BasicMAPF.Instances.Agent;
import BasicMAPF.Instances.InstanceBuilders.InstanceBuilder_ArbitraryGraph;
import BasicMAPF.Instances.InstanceManager;
import BasicMAPF.Instances.InstanceProperties;
import BasicMAPF.Instances.MAPF_Instance;
import BasicMAPF.Instances.Maps.I_Location;
import BasicMAPF.Solvers.ConstraintsAndConflicts.A_Conflict;
import Environment.IO_Package.IO_Manager;
import Environment.Metrics.InstanceReport;
import Environment.Metrics.Metrics;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.BitSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicReference;

/**
 * AUDIT-ONLY, not a permanent regression test. Cross-checks all 5 canonical MUA solvers
 * (CBSStarMUA, AStarODMUAZero/Max/IH/MH) against an independent
 * brute-force joint-state-space oracle for exact optimal MUA*, on a battery of tiny hand-built
 * instances covering the required semantic categories from the audit brief. Produces a printed
 * results table (no hard assertions on solver correctness, by design, so the whole battery runs
 * to completion and prints a full picture even when some solver disagrees with the oracle or
 * times out) plus one sanity assertion at the very end that the oracle itself is internally
 * consistent (never negative, never exceeds the number of UAs).
 *
 * Ground truth used by the oracle (independently re-derived, not copied, from reading
 * SingleAgentPlan.mua()/Solution.MUA() and TransientMAPFSolution.isAchievesTarget()):
 *   - Agent.isUnassigned (source.equals(target)) marks a UA.
 *   - MUA(solution) = number of distinct UA agents whose plan contains >=1 move with prevLocation != currLocation.
 *   - AAs (assigned, source!=target) use TRANSIENT semantics: only need to VISIT their target at some point;
 *     they may leave afterwards. This matches TransientMAPFSettings.defaultTransientMAPF, which is what
 *     CanonicalSolversFactory wires into every one of the 6 solvers under test.
 *   - Standard MAPF joint-move conflict rules apply (vertex conflicts + swapping conflicts), reusing
 *     A_Conflict.conflictBetween(Move,Move) directly from production code so the oracle's notion of
 *     "conflict-free" is byte-for-byte the same as the one solvers/Solution.isValidSolution() use.
 *   - The oracle is a Dijkstra search over (positions, movedUA-bitset, visitedAA-bitset) joint states,
 *     where a transition's cost is the number of UA agents that move (curr!=prev) for the FIRST time in
 *     that step. The search terminates the moment a popped state has all AA agents' visited-bits set;
 *     by Dijkstra's ordering that is the minimum achievable MUA. If OPEN empties out first, the instance
 *     is genuinely infeasible (no joint plan exists at all, independent of MUA).
 */
class MuaCrossSolverAuditTest {

    private static final long ORACLE_STATE_CAP = 3_000_000L;

    /**
     * Ground-truth, oracle-independent confirmation that a UA which moves away and returns to its own original
     * start still contributes exactly 1 to MUA (never 0, never 2+), read directly off SingleAgentPlan.mua()'s
     * actual control flow: it returns 1 on the FIRST move found with prevLocation != currLocation and never
     * re-examines later moves, so a later return to the origin cannot un-set that. This mirrors (independently,
     * not by copying) the same claim already exercised in SolutionMUATest and AStarODMuaSolverTest, and backs
     * the "leaves and returns" category from the audit brief even though no small hand-built instance was found
     * (see the audit report) where such a round trip is the UNIQUELY optimal choice for a pure-MUA objective
     * solver (attempted topologies are consistently either solvable without a return being necessary, or
     * infeasible outright per the oracle - see audit_mua_return_home below).
     */
    @Test
    void uaLeavingAndReturningToOriginalStartStillContributesExactlyOneToMUA() {
        MAPF_Instance instance = loadArbitrary("cbs_star_side_pocket", 3);
        assert instance != null;
        Agent ua = instance.agents.get(2);
        I_Location pocket = instance.map.getMapLocation(ua.source);
        I_Location farEnd = pocket.outgoingEdges().stream()
                .filter(loc -> !loc.equals(instance.map.getMapLocation(instance.agents.get(0).source)))
                .findFirst().orElseThrow();

        SingleAgentPlan roundTrip = new SingleAgentPlan(ua);
        roundTrip.addMove(new Move(ua, 1, pocket, farEnd));
        roundTrip.addMove(new Move(ua, 2, farEnd, pocket)); // returns to its own original start

        Solution solo = new Solution();
        solo.putPlan(roundTrip);
        int mua = solo.MUA();
        System.out.println("[independent-check] UA leaves pocket and returns to it in 2 moves; Solution.MUA()=" + mua + " (expected 1)");
        if (mua != 1) {
            throw new AssertionError("Solution.MUA() reported " + mua + " for a UA that moved away and returned to its own start; expected exactly 1");
        }
    }

    @Test
    void auditAllInstancesAgainstOracleAndAllSolvers() {
        List<String[]> table = new ArrayList<>();
        table.add(new String[]{"instance", "oracle_MUA*", "CBSStarMUA", "Zero", "MAX", "IH", "MH"});

        List<AuditInstance> instances = List.of(
                new AuditInstance("cbs_star_side_pocket (MUA*=1, swap via pocket, 1 UA helps 2 AAs)", "cbs_star_side_pocket", 3, 30_000, 30_000),
                new AuditInstance("mapfua_ma_no_ua_move (MUA*=0)", "mapfua_ma_no_ua_move", 2, 30_000, 30_000),
                new AuditInstance("two_vertex_swap (UNSOLVABLE, no buffer)", "two_vertex_swap", 2, 30_000, 30_000),
                new AuditInstance("audit_mua_side_pocket_idle_ua (2nd UA never needs to move)", "audit_mua_side_pocket_idle_ua", 4, 30_000, 30_000),
                new AuditInstance("audit_mua_return_home (attempted forced-UA-round-trip topology; oracle found it UNSOLVABLE - a second independent unsolvable-case cross-check)", "audit_mua_return_home", 3, 30_000, 30_000),
                new AuditInstance("audit_mua_double_pocket_connected (MUA*=2, connected, 2 distinct UAs each required by a different AA pair)", "audit_mua_double_pocket_connected", 6, 60_000, 25_000),
                new AuditInstance("cbs_star_double_side_pocket (MUA*=2, DISJOINT clusters - known CBS*/MH stress case)", "cbs_star_double_side_pocket", 6, 60_000, 25_000)
        );

        StringBuilder deepDive = new StringBuilder();

        for (AuditInstance ai : instances) {
            MAPF_Instance instance = loadArbitrary(ai.fixturePrefix, ai.agentCount);
            if (instance == null) {
                table.add(new String[]{ai.label, "LOAD-FAILED", "-", "-", "-", "-", "-"});
                continue;
            }

            OracleResult oracle = exactMUA(instance, ORACLE_STATE_CAP);
            String oracleStr = oracle.feasible ? String.valueOf(oracle.mua) : "INFEASIBLE";

            String[] row = new String[7];
            row[0] = ai.label + " [" + instance.agents.size() + " agents]";
            row[1] = oracleStr;

            row[2] = runOneCell(ai, "CBSStarMUA", CanonicalSolversFactory::createCBSStarMUASolver, instance, oracle, ai.normalTimeoutMs, deepDive);
            row[3] = runOneCell(ai, "AStarODMUAZero", CanonicalSolversFactory::createAStarODMUAZeroSolver, instance, oracle, ai.normalTimeoutMs, deepDive);
            row[4] = runOneCell(ai, "AStarODMUAMax", CanonicalSolversFactory::createAStarODMUAMaxSolver, instance, oracle, ai.normalTimeoutMs, deepDive);
            row[5] = runOneCell(ai, "AStarODMUAIH", CanonicalSolversFactory::createAStarODMUAIHSolver, instance, oracle, ai.normalTimeoutMs, deepDive);
            row[6] = runOneCell(ai, "AStarODMUAMH", CanonicalSolversFactory::createAStarODMUAMHSolver, instance, oracle, ai.mhTimeoutMs, deepDive);

            table.add(row);
            System.out.println("[audit] finished instance: " + ai.label + " oracle=" + oracleStr);
            System.out.flush();
        }

        printTable(table);
        if (!deepDive.isEmpty()) {
            System.out.println("\n===== DEEP DIVE ON MISMATCHES/INVALID SOLUTIONS =====");
            System.out.println(deepDive);
        }
    }

    // ---------------------------------------------------------------------------------------------
    // per-cell solver execution with hard wall-clock timeout + validity/MUA cross-check
    // ---------------------------------------------------------------------------------------------

    private String runOneCell(AuditInstance ai, String solverLabel, java.util.function.Supplier<I_Solver> factory,
                               MAPF_Instance instance, OracleResult oracle, long timeoutMs, StringBuilder deepDive) {
        AtomicReference<Solution> solutionRef = new AtomicReference<>();
        AtomicReference<Throwable> errorRef = new AtomicReference<>();
        AtomicReference<InstanceReport> reportRef = new AtomicReference<>();
        long start = System.currentTimeMillis();

        Thread worker = new Thread(() -> {
            try {
                I_Solver solver = factory.get();
                InstanceReport report = Metrics.newInstanceReport();
                Solution sol = solver.solve(instance, new RunParametersBuilder().setTimeout(timeoutMs).setInstanceReport(report).createRP());
                solutionRef.set(sol);
                reportRef.set(report);
            } catch (Throwable t) {
                errorRef.set(t);
            }
        }, "audit-" + solverLabel);
        worker.setDaemon(true);
        worker.start();
        try {
            worker.join(timeoutMs + 15_000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        long elapsed = System.currentTimeMillis() - start;

        if (worker.isAlive()) {
            String msg = String.format("%s on [%s]: HARD TIMEOUT - solver thread still running after %dms (internal timeout was %dms); abandoning thread.",
                    solverLabel, ai.label, elapsed, timeoutMs);
            deepDive.append(msg).append('\n');
            System.out.println("[audit] " + msg);
            return "TIMEOUT(hard)";
        }
        if (errorRef.get() != null) {
            Throwable t = errorRef.get();
            String msg = String.format("%s on [%s]: ERROR - %s: %s", solverLabel, ai.label, t.getClass().getSimpleName(), t.getMessage());
            deepDive.append(msg).append('\n');
            for (StackTraceElement ste : Arrays.copyOf(t.getStackTrace(), Math.min(5, t.getStackTrace().length))) {
                deepDive.append("    at ").append(ste).append('\n');
            }
            System.out.println("[audit] " + msg);
            return "ERROR(" + t.getClass().getSimpleName() + ")";
        }

        Solution solution = solutionRef.get();
        InstanceReport report = reportRef.get();
        if (solution == null) {
            Integer elapsedReported = report != null ? report.getIntegerValue(InstanceReport.StandardFields.elapsedTimeMS) : null;
            boolean looksLikeTimeout = elapsedReported != null && elapsedReported >= (timeoutMs - Math.max(500, timeoutMs / 20));
            if (oracle.feasible) {
                String verdict = looksLikeTimeout ? "TIMEOUT" : "FAIL(no-solution,oracle-says-feasible)";
                String msg = String.format("%s on [%s]: returned NULL (elapsed=%sms, timeoutThreshold=%dms) but oracle says feasible with MUA*=%d -> %s",
                        solverLabel, ai.label, elapsedReported, timeoutMs, oracle.mua, verdict);
                deepDive.append(msg).append('\n');
                System.out.println("[audit] " + msg);
                return verdict;
            } else {
                return "PASS(correctly-unsolvable)";
            }
        }

        // solver claims solved. Independently validate: conflict-free, AA transient-visits target, UA/AA semantics.
        String validity = independentlyValidate(instance, solution);
        int solverMua = solution.MUA();

        if (!oracle.feasible) {
            String msg = String.format("%s on [%s]: returned a SOLUTION (MUA=%d, validity=%s) but oracle says INFEASIBLE. CRITICAL.",
                    solverLabel, ai.label, solverMua, validity);
            deepDive.append(msg).append('\n');
            System.out.println("[audit] " + msg);
            return "FAIL(solved-but-oracle-infeasible,MUA=" + solverMua + "," + validity + ")";
        }

        if (!validity.equals("VALID")) {
            String msg = String.format("%s on [%s]: returned an INVALID solution: %s (reported MUA=%d, oracle MUA*=%d)",
                    solverLabel, ai.label, validity, solverMua, oracle.mua);
            deepDive.append(msg).append('\n');
            System.out.println("[audit] " + msg);
            return "FAIL(INVALID:" + validity + ")";
        }

        if (solverMua != oracle.mua) {
            String msg = String.format("%s on [%s]: MISMATCH - solver MUA=%d, oracle MUA*=%d (solution was structurally VALID)",
                    solverLabel, ai.label, solverMua, oracle.mua);
            deepDive.append(msg).append('\n');
            System.out.println("[audit] " + msg);
            return "FAIL(" + solverMua + "!=" + oracle.mua + ")";
        }

        return "PASS(" + solverMua + "," + elapsed + "ms)";
    }

    /**
     * Validates: conflict-free (isValidSolution), every agent's plan starts at its source, every AA's plan
     * contains its target at some point (transient visit-only semantics per SingleAgentPlan.containsTarget()),
     * and every consecutive pair of moves is to a graph-neighbor or a self-stay. Deliberately implemented
     * independently of Solution.solves()/TransientMAPFSolution so it doesn't inherit whatever solver-returned
     * Solution subtype happened to come back.
     */
    private String independentlyValidate(MAPF_Instance instance, Solution solution) {
        A_Conflict conflict = solution.firstConflict(false, false);
        if (conflict != null) {
            return "CONFLICT@t=" + conflict.time + " agents=" + conflict.agent1.iD + "," + conflict.agent2.iD;
        }
        for (Agent agent : instance.agents) {
            SingleAgentPlan plan = solution.getPlanFor(agent);
            if (plan == null) return "MISSING-PLAN(agent" + agent.iD + ")";
            if (plan.size() > 0) {
                I_Location expectedSource = instance.map.getMapLocation(agent.source);
                if (!plan.getFirstMove().prevLocation.equals(expectedSource)) {
                    return "WRONG-SOURCE(agent" + agent.iD + ")";
                }
                Move prev = null;
                for (Move m : plan) {
                    if (prev != null) {
                        if (!prev.currLocation.equals(m.prevLocation)) return "DISCONNECTED-PLAN(agent" + agent.iD + ")";
                        if (m.timeNow - prev.timeNow != 1) return "NON-CONSECUTIVE-TIME(agent" + agent.iD + ")";
                    }
                    if (!m.prevLocation.equals(m.currLocation) && !m.prevLocation.isNeighbor(m.currLocation)) {
                        return "NON-NEIGHBOR-MOVE(agent" + agent.iD + " " + m + ")";
                    }
                    prev = m;
                }
            }
            if (!agent.isUnassigned && !plan.containsTarget()) {
                return "AA-NEVER-VISITED-TARGET(agent" + agent.iD + ")";
            }
        }
        return "VALID";
    }

    // ---------------------------------------------------------------------------------------------
    // Independent brute-force oracle: Dijkstra over (positions, movedUA, visitedAA) joint states.
    // ---------------------------------------------------------------------------------------------

    private record OracleResult(boolean feasible, int mua, long statesExpanded) {}

    private OracleResult exactMUA(MAPF_Instance instance, long stateCap) {
        List<Agent> agents = instance.agents;
        int n = agents.size();
        I_Location[] start = new I_Location[n];
        boolean[] isUA = new boolean[n];
        for (int i = 0; i < n; i++) {
            start[i] = instance.map.getMapLocation(agents.get(i).source);
            isUA[i] = agents.get(i).isUnassigned;
        }

        PriorityQueue<OracleNode> open = new PriorityQueue<>((a, b) -> {
            int byCost = Integer.compare(a.g, b.g);
            return byCost != 0 ? byCost : Long.compare(a.serial, b.serial);
        });
        Map<OracleKey, Integer> bestG = new HashMap<>();
        long[] serialGen = {0};
        long expanded = 0;

        OracleNode root = new OracleNode(start, new BitSet(n), new BitSet(n), 0, serialGen[0]++);
        open.add(root);
        bestG.put(root.key(), 0);

        while (!open.isEmpty()) {
            if (++expanded > stateCap) {
                throw new IllegalStateException("Oracle exceeded state cap " + stateCap + " without converging - instance too large for brute force.");
            }
            OracleNode current = open.poll();
            Integer knownBest = bestG.get(current.key());
            if (knownBest == null || knownBest < current.g) continue; // stale entry

            if (allAAVisited(agents, current.visited)) {
                return new OracleResult(true, current.g, expanded);
            }

            List<I_Location[]> jointMoves = new ArrayList<>();
            enumerateJointMoves(current.positions, 0, new I_Location[n], jointMoves);

            for (I_Location[] next : jointMoves) {
                BitSet moved = (BitSet) current.moved.clone();
                BitSet visited = (BitSet) current.visited.clone();
                int addedCost = 0;
                for (int i = 0; i < n; i++) {
                    boolean didMove = !next[i].equals(current.positions[i]);
                    if (didMove && isUA[i] && !moved.get(i)) {
                        moved.set(i);
                        addedCost++;
                    }
                    if (!isUA[i] && next[i].getCoordinate().equals(agents.get(i).target)) {
                        visited.set(i);
                    }
                }
                int childG = current.g + addedCost;
                OracleNode child = new OracleNode(next, moved, visited, childG, serialGen[0]++);
                Integer old = bestG.get(child.key());
                if (old == null || childG < old) {
                    bestG.put(child.key(), childG);
                    open.add(child);
                }
            }
        }
        return new OracleResult(false, -1, expanded);
    }

    private boolean allAAVisited(List<Agent> agents, BitSet visited) {
        for (int i = 0; i < agents.size(); i++) {
            if (!agents.get(i).isUnassigned && !visited.get(i)) return false;
        }
        return true;
    }

    /**
     * Enumerates all conflict-free joint single-timestep moves from {@code current}, using the SAME conflict
     * definition as production code (A_Conflict.conflictBetween on synthetic Move objects at a shared dummy
     * timestep), rather than a hand-rolled reimplementation of vertex/swap conflict rules.
     */
    private void enumerateJointMoves(I_Location[] current, int index, I_Location[] chosen, List<I_Location[]> result) {
        if (index == current.length) {
            result.add(Arrays.copyOf(chosen, chosen.length));
            return;
        }
        List<I_Location> options = new ArrayList<>(current[index].outgoingEdges());
        options.add(current[index]); // stay
        for (I_Location next : options) {
            boolean conflict = false;
            for (int j = 0; j < index; j++) {
                Move dummyThis = dummyMove(index, 1, current[index], next);
                Move dummyOther = dummyMove(j, 1, current[j], chosen[j]);
                if (A_Conflict.haveConflicts(dummyThis, dummyOther)) {
                    conflict = true;
                    break;
                }
            }
            if (!conflict) {
                chosen[index] = next;
                enumerateJointMoves(current, index + 1, chosen, result);
            }
        }
    }

    /** A Move-alike using a shared placeholder Agent per index so A_Conflict's equals-ignoring-agent logic still discriminates agents correctly via location/time only (conflict checks never compare .agent, only prevLocation/currLocation/timeNow), avoiding needing real Agent objects with matching source/target semantics for this pure geometry check. */
    private static Move dummyMove(int agentIndex, int timeNow, I_Location prev, I_Location curr) {
        Agent placeholder = new Agent(agentIndex, prev.getCoordinate(), prev.getCoordinate());
        return new Move(placeholder, timeNow, prev, curr);
    }

    private record OracleNode(I_Location[] positions, BitSet moved, BitSet visited, int g, long serial) {
        OracleKey key() { return new OracleKey(positions, moved, visited); }
    }

    private record OracleKey(I_Location[] positions, BitSet moved, BitSet visited) {
        OracleKey {
            positions = Arrays.copyOf(positions, positions.length);
            moved = (BitSet) moved.clone();
            visited = (BitSet) visited.clone();
        }
        @Override public boolean equals(Object o) {
            return o instanceof OracleKey that
                    && Arrays.equals(positions, that.positions)
                    && moved.equals(that.moved)
                    && visited.equals(that.visited);
        }
        @Override public int hashCode() {
            int result = Arrays.hashCode(positions);
            result = 31 * result + moved.hashCode();
            result = 31 * result + visited.hashCode();
            return result;
        }
    }

    // ---------------------------------------------------------------------------------------------
    // instance loading + misc plumbing
    // ---------------------------------------------------------------------------------------------

    private record AuditInstance(String label, String fixturePrefix, int agentCount, long normalTimeoutMs, long mhTimeoutMs) {}

    private MAPF_Instance loadArbitrary(String prefix, int agents) {
        String dir = IO_Manager.buildPath(new String[]{IO_Manager.testResources_Directory, "Instances", "ArbitraryGraphs"});
        InstanceManager.Moving_AI_Path path = new InstanceManager.Moving_AI_Path(
                IO_Manager.buildPath(new String[]{dir, prefix + ".edgelist"}),
                IO_Manager.buildPath(new String[]{dir, prefix + ".scene"}));
        for (int i = 0; i < 3; i++) {
            InstanceBuilder_ArbitraryGraph builder = new InstanceBuilder_ArbitraryGraph();
            builder.prepareInstances(prefix, path, new InstanceProperties(null, -1, new int[]{agents}));
            MAPF_Instance instance = builder.getNextExistingInstance();
            if (instance != null) return instance;
        }
        return null;
    }

    private void printTable(List<String[]> table) {
        int cols = table.get(0).length;
        int[] widths = new int[cols];
        for (String[] row : table) {
            for (int c = 0; c < cols; c++) {
                widths[c] = Math.max(widths[c], row[c] == null ? 4 : row[c].length());
            }
        }
        StringBuilder sb = new StringBuilder("\n===== MUA CROSS-SOLVER AUDIT RESULTS =====\n");
        for (String[] row : table) {
            for (int c = 0; c < cols; c++) {
                sb.append(String.format("%-" + (widths[c] + 2) + "s", row[c]));
            }
            sb.append('\n');
        }
        System.out.println(sb);
    }
}
