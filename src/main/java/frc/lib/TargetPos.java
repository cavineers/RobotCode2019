package frc.lib;

import frc.lib.pathPursuit.Point;

public class TargetPos {
    double x;
    double y;
    double theta;
    int updateNum;
    double timestamp;

    /**
     * A helper class dedicated to storing information about a target
     * 
     * @param x x position of the target relative to the robot
     * @param y y position of the target relative to the robot
     * @param theta angle of the target relative to the robot
     * @param updateNum the positive whole number describing the number of the update
     * @param timestamp time the target was aquired
     */
    public TargetPos(double x, double y, double theta, int updateNum, double timestamp) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.updateNum = updateNum;
        this.timestamp = timestamp;
    }

    public double getX() {
        return x;
    }
    
    public double getY() {
        return y;
    }

    public Point getPosition() {
        return new Point(this.getX(), this.getY());
    }

    public double getAngle() {
        return theta;
    }

    public int updateNum() {
        return updateNum;
    }

    public double getTimestamp() {
        return timestamp;
    }
}