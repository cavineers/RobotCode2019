package frc.lib.pathPursuit;

import frc.robot.Constants;
import frc.lib.MathHelper;
import frc.lib.RobotPos;

public class ConnectionArc {
    Point startPoint;
    Point endPoint;
    double radius;
    boolean isTurningRight;
    boolean isReversed;

    public ConnectionArc(RobotPos rPos, Point endPoint, boolean isReversed) {
        this.startPoint = rPos.position;
        this.endPoint = endPoint;
        this.isReversed = isReversed;
        Point robotRelativeGoal;
        
        if (this.isReversed) {
            robotRelativeGoal = Point.getP1RelativeToP2(this.endPoint, this.startPoint, rPos.heading - Math.PI);
        } else {
            robotRelativeGoal = Point.getP1RelativeToP2(this.endPoint, this.startPoint, rPos.heading);
        }
        
        if (MathHelper.areApproxEqual(robotRelativeGoal.getX(), 0) && robotRelativeGoal.getY() > 0) { 
            //The robot is co-linear with the target and the point is in front of the robot
            this.radius = Double.MAX_VALUE;
        } else if (MathHelper.areApproxEqual(robotRelativeGoal.getX(), 0) && robotRelativeGoal.getY() < 0) {
            //The robot is co-linear with the target and the point is in behind the robot, conduct a tight turn around
            this.radius = 1;
        }  else {
            //The robot should turn to face the next point
            this.radius = Math.pow(robotRelativeGoal.getHypot(), 2) / (2 * Math.abs(robotRelativeGoal.getX()));
        }
        
        if (robotRelativeGoal.getX() < 0) {
            this.isTurningRight = true;
        } else {
            this.isTurningRight = false;
        }
    }

    public ConnectionArc(double radius, boolean isTurningRight) {
        this.radius = radius;
        this.isTurningRight = isTurningRight;
    }

    public double getLength() {
        return MathHelper.getAngleForArc(startPoint, endPoint, radius);
    }

    /**
     * 
     * @param currentVel:
     *            the current center velocity of the robot
     * @param turnRad:
     *            the radius of the turn desired
     * @return a left wheel velocity that will allow the robot to make the turn
     */
    public double getRightVelocityTarget(double currentVel) {
        double a;
        if (this.isTurningRight) {
            a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
        } else {
            a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
        }
        return (2 * currentVel) / (a + 1);
    }

    /**
     * 
     * @param currentVel:
     *            the current center velocity of the robot
     * @param turnRad:
     *            the radius of the turn desired
     * @return: a right wheel velocity that will allow the robot to make the
     *          turn
     */
    public double getLeftVelocityTarget(double currentVel) {
        double a;
        if (this.isTurningRight) {
            a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
        } else {
            a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
        }
        return (2 * currentVel) / (a + 1);

    }

    public double getLeftVelocityTargetFromRightVelocity(double rVel) {
        double a;
        if (this.isTurningRight) {
            a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
        } else {
            a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
        }
        return a * rVel;
    }

    public double getRightVelocityTargetFromLeftVelocity(double lVel) {
        double a;
        if (this.isTurningRight) {
            a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
        } else {
            a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
        }
        return a * lVel;
    }

}
