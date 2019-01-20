package frc.lib.dubinPath;

public class Control {
    public double steeringAngle;
    public double timesteps;
    public double steeringRadius;
    public double goalHeading;

    public Control() {
        this.steeringRadius = 0.0; // Positive means to the left, negative to the right
        //		this.goalHeading = 0.0;
        this.steeringAngle = 0.0;
        this.timesteps = 0.0; // TODO: Find something better than this. Is this going wrong right now?
    }
}