package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

final class MuaHeuristicMetrics {
    long evaluations;
    long internalGenerated;
    long internalExpanded;
    long lazyFullEvaluations;
    long lazyInheritedEvaluations;

    void addInternalGenerated(long delta) {
        internalGenerated += delta;
    }

    void addInternalExpanded(long delta) {
        internalExpanded += delta;
    }
}
