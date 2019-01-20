package frc.lib;

import frc.lib.MathHelper;
import frc.lib.pathPursuit.Point;

public class RobotPos {
    public Point position;
    public double heading; //heading in radians
    double rVel;
    double lVel;
    
    public RobotPos(double robotX, double robotY, double heading, double rVel, double lVel) { //heading in radians
        this(new Point(robotX, robotY), heading, rVel, lVel);
    }
    
    public RobotPos(Point pos, double heading, double rVel, double lVel) { // heading in radians
        this.position = pos;
        this.heading = heading;
        this.rVel = rVel;
        this.lVel = lVel;
    }

    public double getX() {
        return position.getX();
    }

    public double getY() {
        return position.getY();
    }
    
    public double getRightVel() {
        return this.rVel;
    }
    
    public double getLeftVel() {
        return this.lVel;
    }
    
    public double getVelocity() {
        return (this.rVel + this.lVel) / 2.0;
    }
    
    public void setHeading(double heading) {
        this.heading = MathHelper.angleToNegPiToPi(heading);
    }
    
    public double getHeading() {
        return this.heading;
    }

    public void setVelocities(double rVel, double lVel) {
        this.rVel = rVel;
        this.lVel = lVel;
    }
    
    /**
     * Gives an approximation of the robot's position at the next update
     * 
     * @param dt - change in time between updates
     * @return - expected position
     */
    public Point getVelocityLookaheadPoint(double dt) {
        // get where the robot will be next update
        double dist = dt * this.getVelocity();
        Point lookahead = new Point(Math.cos(heading) * dist, Math.sin(heading) * dist);
        // add that point to the robot's current position
        return Point.addPoints(this.position, lookahead);
    }
    
    @Override
    public String toString() {
        return this.position.getX() + "," + this.position.getY() + "," + this.heading + "," + this.lVel + "," + this.rVel; 
    }
}
