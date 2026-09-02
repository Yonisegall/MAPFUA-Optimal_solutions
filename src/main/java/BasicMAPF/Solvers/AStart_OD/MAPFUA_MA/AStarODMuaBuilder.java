package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

import TransientMAPF.TransientMAPFSettings;

public class AStarODMuaBuilder {
    private MapfuaObjective objective = MapfuaObjective.MUA;
    private MuaHeuristicType heuristicType = MuaHeuristicType.ZERO;
    private boolean lazyEvaluation = false;
    private TransientMAPFSettings transientMAPFSettings = TransientMAPFSettings.defaultTransientMAPF;

    public AStarODMuaBuilder setObjective(MapfuaObjective objective) {
        this.objective = objective;
        return this;
    }

    public AStarODMuaBuilder setHeuristicType(MuaHeuristicType heuristicType) {
        this.heuristicType = heuristicType;
        return this;
    }

    public AStarODMuaBuilder setLazyEvaluation(boolean lazyEvaluation) {
        this.lazyEvaluation = lazyEvaluation;
        return this;
    }

    public AStarODMuaBuilder setTransientMAPFSettings(TransientMAPFSettings transientMAPFSettings) {
        this.transientMAPFSettings = transientMAPFSettings;
        return this;
    }

    public AStarODMuaSolver createSolver() {
        return new AStarODMuaSolver(objective, heuristicType, lazyEvaluation, transientMAPFSettings);
    }
}
