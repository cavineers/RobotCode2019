package frc.lib.pathPursuit;

import frc.lib.RobotPos;
import frc.robot.Constants;

public class VelocityManager { //creates a trapezoidal velocity curve for the robot to follow
    public double dt;
    double lastUpdateTime;
    
    boolean isHeadingFrozen;
    double frozenHeading = 0;
    
    // should recalculate the dt every cycle - true when on a robot, false when in debug mode
    boolean recalcDt = false;
    boolean isReversed = false;
    double maxAccel = 0;

    Lookahead lookahead;
    
    public VelocityManager(boolean isDebug, boolean isReversed, double maxAccel, Lookahead lookahead) {
        this.recalcDt = !isDebug;
        this.isReversed = isReversed;
        this.maxAccel = maxAccel;
        this.dt = -1;
        this.lastUpdateTime = -1;
        this.isHeadingFrozen = false;
        this.lookahead = lookahead;
    }
    
    /**
     * returns a RobotCmd containing the desired left and right velocities for the wheels
     * 
     * @param segment: the current segment the robot is following
     * @param state: the current position + velocity of the robot
     */
    public RobotCmd getVelCmd(Segment segment, RobotPos state, Point lookaheadPoint) {
        if (dt == -1) { 
            //on the first update, setup the dt calculations
            lastUpdateTime = System.currentTimeMillis();
            this.dt = Constants.kDefaultDt;
        } else if (this.recalcDt){ 
            // update dt based on time elapsed since last update
            this.updateDt(); 
        }
        double overallVel = this.getOverallVelocityTarget(segment, state);
        
        if (this.isHeadingFrozen) {
            return new RobotCmd(overallVel, overallVel);
        }
        ConnectionArc arc = new ConnectionArc(state, lookaheadPoint, isReversed);
        
        double rVel = arc.getRightVelocityTarget(overallVel);
        double lVel = arc.getLeftVelocityTarget(overallVel);
        
        if (isReversed) {  //when isReversed, flip wheel velocities
            return new RobotCmd(rVel, lVel);
        } else {
            return new RobotCmd(lVel, rVel);
        }
    }
    
    
    /**
     * returns the velocity desired for the robot as a whole
     * 
     * @param segment: the current segment the robot is following
     * @param state: the current position + velocity of the robot
     */
    public double getOverallVelocityTarget(Segment segment, RobotPos state) {
        
        if (this.isHeadingFrozen) {
            this.getNextVelocity(this.maxAccel, state.getVelocity(), 0);
        }

        if (!segment.isAcceleratingToEndpoint() && this.shouldAccelToEndPoint(segment, state)) {
            //The robot must accelerate/decelerate to reach the velocity desired by the segment's endpoint
            segment.setIsAcceleratingToEndpoint(true);
        }
        if (segment.isAcceleratingToEndpoint()) {
            double requiredAccelToFinish;
            if (segment.getEndVelocity() == 0) { //The robot will stop at the end of this segment, try to stop exactly at the end
                if (isReversed) {
                    requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), -1 * segment.getEndVelocity(), segment.getDistanceToEndpoint(segment.getClosestPointOnSegment(state.getVelocityLookaheadPoint(dt))));
                } else {
                    requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), segment.getEndVelocity(), segment.getDistanceToEndpoint(segment.getClosestPointOnSegment(state.getVelocityLookaheadPoint(dt))));
                }
            }
            else { //The robot will be continuing onto another segment, reach final velocity before it switches segments
                double currentLookahead = this.lookahead.getLookaheadForSpeed(state.getVelocity());
                double distToCompletion = segment.getDistanceToEndpoint(segment.getClosestPointOnSegment(state.getVelocityLookaheadPoint(dt))) - Constants.kPathPursuitTolerance - currentLookahead;
                if (distToCompletion < 0) {
                    if (isReversed) {
                        return this.getNextVelocity(this.maxAccel, state.getVelocity(), -1 * segment.getEndVelocity());
                    } else {
                        return this.getNextVelocity(this.maxAccel, state.getVelocity(), segment.getEndVelocity());
                    }
                }
                if (isReversed) {
                    requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), -1 * segment.getEndVelocity(), distToCompletion);

                } else {
                    requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), segment.getEndVelocity(), distToCompletion);
                }
            }
            if (isReversed) {
                return this.getNextVelocity(requiredAccelToFinish, state.getVelocity(), -1 * segment.getEndVelocity());
            } else{
                return this.getNextVelocity(requiredAccelToFinish, state.getVelocity(), segment.getEndVelocity());
            }
            
        }
        else {
            //The robot must accelerate/decelerate to reach the max speed of the path
            if (isReversed) {
                return this.getNextVelocity(this.maxAccel, state.getVelocity(), -1 * segment.getMaxVelocity());
            } else {
                return this.getNextVelocity(this.maxAccel, state.getVelocity(), segment.getMaxVelocity());
            }
            
        }
    }
    
    /**
     * returns if the robot should accelerate/decelerate to get to the velocity desired at the endpoint
     * 
     * @param segment: the segment the robot is currently following
     * @param state: the current position + velocity of the robot
     */
    public boolean shouldAccelToEndPoint(Segment segment, RobotPos state) {
        if (segment.getEndVelocity() == segment.getMaxVelocity()) {
            return false;
        }
        double requiredAccelToFinish;
        if (segment.getEndVelocity() == 0) { //The robot will stop at the end of this segment, try to stop exactly at the end
            requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), segment.getEndVelocity(), segment.getDistanceToEndpoint(segment.getClosestPointOnSegment(state.getVelocityLookaheadPoint(dt))));
        } else { //The robot will be continuing onto another segment, reach final velocity before it does
            double currentLookahead = this.lookahead.getLookaheadForSpeed(state.getVelocity());
            double distToCompletion = segment.getDistanceToEndpoint(segment.getClosestPointOnSegment(state.getVelocityLookaheadPoint(dt))) - Constants.kPathPursuitTolerance - currentLookahead;
            if (distToCompletion < 0) {
                return true;
            }
            if (isReversed) {
                requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), -1 * segment.getEndVelocity(), distToCompletion);
            } else { 
                requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), segment.getEndVelocity(), distToCompletion);
            }
                        
            }
        return requiredAccelToFinish >= this.maxAccel;
    }
    
    /**
     * returns the acceleration required to achieve a certain velocity from the current velocity 
     * over a given distance
     * 
     * @param v1: current velocity
     * @param v2: desired velocity
     * @param dist: distance desired
     */
    public double getAccelNeededToGetToVelByPoint(double v1, double v2, double dist) {
        double deltaV = v2 - v1;
        double accel = -Math.pow(deltaV, 2) / (2 * dist);
        return Math.abs(accel);
    }
    
    
    /**
     * Returns the closest velocity the robot can get to the desired velocity
     * without violating maximum acceleration constraints
     * 
     * @param currentVel: the current velocity of the robot
     * @param desiredVel: the desired velocity of the robot
     * @param maxSpeedAccel: the max acceleration allocated to changing robot speed
     */
    public double getNextVelocity(double maxSpeedAccel, double currentVel, double desiredVel) {
        double delta = desiredVel - currentVel;
        if (Math.abs(delta) <= (this.dt * maxSpeedAccel)) {
            // if moving to the desired velocity is allowed within acceleration constraints, 
            // return the desired velocity
            return desiredVel;
            
        } else if (delta > 0) {
            // The next velocity is greater than the current velocity, speed up the robot
            return currentVel + (this.dt * maxSpeedAccel);
        } else if (delta < 0) {
            // The next velocity is less than the current velocity, slow down the robot
            return currentVel - (this.dt * maxSpeedAccel);
        } else {
            // The next velocity is the same as the current velocity, maintain current velocity
            return currentVel;
        }
    }
    
    /**
     * Update self.dt with the current change in time between updates
     */
    public void updateDt() {
        this.dt = (System.currentTimeMillis() - lastUpdateTime) / 1000;
        this.lastUpdateTime = System.currentTimeMillis();
    }
    
    /**
     * Stop the robot from changing heading
     * Prevents the robot from turning around if it overshoots the end of the last segment
     * 
     * @param heading: the heading to maintain for the remainder of the path
     */
    public void freezeHeading(double heading) {
        this.isHeadingFrozen = true;
        this.frozenHeading = heading;
    }
    
}
