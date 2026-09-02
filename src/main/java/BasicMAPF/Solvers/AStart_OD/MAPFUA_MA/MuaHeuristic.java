package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

interface MuaHeuristic {
    long estimate(MuaSearchState state);
    String name();
    MuaHeuristicMetrics metrics();
}
