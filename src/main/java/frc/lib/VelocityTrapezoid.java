package frc.lib;

public class VelocityTrapezoid {
    double maxAccel = 0;
    double maxSpeed = 0;
    double dt = 0;
    double lastUpdateTime = 0;
    boolean isAcceleratingToEndpoint = false;
    boolean isUpdatingDt = true;

    boolean doesLockDecel = true;

    public VelocityTrapezoid(double maxAccel, double maxSpeed, double dt) {
        this.maxAccel = Math.abs(maxAccel);
        this.maxSpeed = Math.abs(maxSpeed);
        this.dt = dt;
        this.lastUpdateTime = System.currentTimeMillis();
    }

    public void setDebugMode() {
        isUpdatingDt = false;
    }

    public void setDecelLock(boolean lock) {
        doesLockDecel = lock;
    }
    
    /**
     * Update the velocity profile with new information
     * 
     * @param currentVel the current velocity of the system
     * @param distanceRemaining the distance to the endpoint
     * @return the velocity that the system should be traveling
     */
    public double update(double currentVel, double distanceRemaining) {
        if (isUpdatingDt) {
            //recalc dt every interval (time in between updates)
            updateDt();
        }

        //calculate the acceleration needed for the robot to stop by the given endpoint
        double endpointAccel = this.getAccelNeededToStopByPoint(currentVel, distanceRemaining);

        if (endpointAccel > this.maxAccel || this.isAcceleratingToEndpoint) {
            if (this.doesLockDecel) {
                this.isAcceleratingToEndpoint = true;
            }
            if (distanceRemaining > 0) {
                // if the robot cannot reach the endpoint within max acceleration, slow down as fast as is required
                return this.getNextVelocity(currentVel, 0, endpointAccel);
            } else {
                return this.getNextVelocity(currentVel, 0, endpointAccel);
            }
        } else {
            if (distanceRemaining > 0) {
                // otherwise, speed up to max speed and maintain it
                return this.getNextVelocity(currentVel, this.maxSpeed, this.maxAccel);
            } else {
                return this.getNextVelocity(currentVel, -this.maxSpeed, this.maxAccel);
            }
        }
    }

    /**
     * returns the acceleration required to achieve a certain velocity from the current velocity 
     * over a given distance
     * 
     * @param vel current velocity
     * @param dist distance left to drive
     */
    private double getAccelNeededToStopByPoint(double vel, double dist) {
        double accel = -Math.pow(vel, 2) / (2 * dist);
        return Math.abs(accel);
    }

    /**
     * Returns the closest velocity the robot can get to the desired velocity
     * without violating maximum acceleration constraints
     * 
     * @param currentVel the current velocity of the robot
     * @param desiredVel the desired velocity of the robot
     * @param maxAccel the max acceleration allocated to changing robot speed
     */
    private double getNextVelocity(double currentVel, double desiredVel, double maxAccel) {
        double delta = desiredVel - currentVel;

        if (Math.abs(delta) <= (this.dt * maxAccel)) {
            // if moving to the desired velocity is allowed within acceleration constraints, 
            // return the desired velocity
            return desiredVel;
        } else if (delta > 0) {
            // The next velocity is greater than the current velocity, speed up the robot
            return currentVel + (this.dt * maxAccel);
        } else if (delta < 0) {
            // The next velocity is less than the current velocity, slow down the robot
            return currentVel - (this.dt * maxAccel);
        } else {
            // The next velocity is the same as the current velocity, maintain current velocity
            return currentVel;
        }
    }

    /**
     * Update self.dt with the current change in time between updates
     */
    private void updateDt() {
        this.dt = (System.currentTimeMillis() - lastUpdateTime) / 1000;
        this.lastUpdateTime = System.currentTimeMillis();
    }

}