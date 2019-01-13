package frc.lib.pathPursuit;

public class RobotCmd {
    double lVelDesired;
    double rVelDesired;
    
    public RobotCmd(double lVelDesired, double rVelDesired) {
        this.lVelDesired = lVelDesired;
        this.rVelDesired = rVelDesired;
    }
    
    public double getLeftVel() {
        return this.lVelDesired;
    }
    
    public double getRightVel() {
        return this.rVelDesired;
    }
    
    @Override
    public String toString() {
        return "left:" + this.lVelDesired + " right:" + this.rVelDesired;
    }
    
}
