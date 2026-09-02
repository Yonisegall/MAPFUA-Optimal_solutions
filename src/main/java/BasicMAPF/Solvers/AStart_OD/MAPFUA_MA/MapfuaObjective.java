package BasicMAPF.Solvers.AStart_OD.MAPFUA_MA;

public enum MapfuaObjective {
    MUA("MUA"),
    FUEL("FUEL"),
    SST("SST");

    public final String reportName;

    MapfuaObjective(String reportName) {
        this.reportName = reportName;
    }

    public long stepCost(int newMovedUAs, int fuel, int sst) {
        return switch (this) {
            case MUA -> newMovedUAs;
            case FUEL -> fuel;
            case SST -> sst;
        };
    }
}
