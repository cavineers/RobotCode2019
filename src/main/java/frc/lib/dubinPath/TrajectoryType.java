package frc.lib.dubinPath;

public enum TrajectoryType{ //for Dubin's curves
    LRL(0),
    RLR(1),
    LSL(2),
    LSR(3),
    RSL(4),
    RSR(5);

    private int value;

    private TrajectoryType(int value){
        this.value = value;
    }
}