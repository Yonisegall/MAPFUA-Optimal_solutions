package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

/**
 * The A*OD MAPFUA heuristics from the source paper, plus the uninformed baseline.
 * cap(ai) is the minimum number of distinct currently-unmoved UAs that assigned agent ai
 * must capture (remove) on its way to its target.
 */
public enum MuaHeuristicType {
    /** h = 0 (uninformed A*). */
    ZERO,
    /** MAX: max over active AAs of cap(ai), each AA evaluated independently. */
    MAX,
    /** IH (Increasing Heuristic): smallest k such that removing some k unmoved UAs lets every active AA independently reach its target. */
    INCREASING,
    /** MH (Merge Heuristic): all active AAs merged into one meta-agent; joint search minimizing distinct captured UAs. */
    MERGE
}
