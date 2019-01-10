package frc.lib;

import frc.lib.Vector;
import frc.lib.pathPursuit.Point;
import frc.robot.Constants;
import frc.robot.Robot;

public class CoordinateFrameHelper {
    //TODO: test this class

    /**
     * Get the vector on the field coordinate plane representing the difference between
     * the the robot's center and the camera
     * 
     * @return a vector representing the distance from the the camera to the center of the robot
     */
    public static Vector getFieldRelativeCamera() {
        return Constants.cameraRelativeToRobotCenter.rotate(Robot.getAngleRad());
    }
    
    /**
     * Turns camera centered odometry data and the angle of the target into robot centered data.  This gives the position of the robot
     * relative to the vision target
     * 
     * @param posFromCamera odometry point from the camera
     * @param targetAngle the angle of the target in the real world
     * @return the robot's position (relative to the camera target)
     */
    public static Point convertCameraRelativePosToRobotRelativePos(Point posFromCamera, double targetAngle) {
        Vector cameraToTarget = posFromCamera.getVector();
        cameraToTarget = cameraToTarget.rotate(targetAngle);

        Vector robotToTarget = Vector.add(cameraToTarget, getFieldRelativeCamera());

        return robotToTarget.getPoint();
    }

    /**
     * Turns the position of a robot relative to a target alligned with the field and the position of that target
     * in the xy plane into the absolute position of the robot
     * 
     * @param targetRelRobotPos the robot position relative to the target
     * @param targetPos the position of the target on the field
     */
    public static Point convertRobotRelativeToTargetToAbsoluteRobotPos(Point targetRelRobotPos, Point targetPos) {
        Vector targetToRobot = targetRelRobotPos.getVector();
        Vector originToTarget = targetPos.getVector();

        Vector originToRobot = Vector.subtract(originToTarget, targetToRobot);
        return originToRobot.getPoint();
    }

}
