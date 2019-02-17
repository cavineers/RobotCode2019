package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.RobotPos;
import frc.lib.TargetUpdate;
import frc.lib.Vector2D;
import frc.lib.Vector3D;
import frc.lib.dubinPath.DubinPathCalculator;
import frc.lib.dubinPath.DubinsPath;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.RobotCmd;
import frc.robot.Robot;
import frc.robot.Constants;

public class TargetVisionTape extends Command {

    Path path;
    boolean forceFinish;

    public TargetVisionTape() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        this.setTimeout(10); // set the command timeout to 10 seconds
        path = null;
        forceFinish = false;
        if (Robot.reflectiveTapeCamera.getUpdate() == null) {
            forceFinish = true; //stop if there is no camera update
            return;
        }
        // otherwise generate a path using this vision data

        //attempt to obtain the image data from the camera
        TargetUpdate targetUpdate = null; // data received from the camera
        targetUpdate = Robot.reflectiveTapeCamera.getUpdate();

        //otherwise, generate a path with that info
        this.path = this.createPath(targetUpdate);
        if (path == null) {
            forceFinish = true;
        }
    }

    protected Path createPath(TargetUpdate targetUpdate) {
        System.out.println("Found Target");
        SmartDashboard.putString("target status", "found target");

        RobotPos robotFieldPos = Robot.estimator.getPositionAtTime(targetUpdate.getTimestamp()); // the position of the robot when the picture was taken (field coords)
        
        System.out.println("Pos at time: " + robotFieldPos);
        SmartDashboard.putString("Pos at Time", robotFieldPos.toString());

        //convert the target heading vector to the robot's coordinate system
        Vector3D targetHeadingVect = targetUpdate.getHeadingVector();
        targetHeadingVect.rotate(Constants.kCameraToRobotMatrix);

        System.out.println("Target Heading Vect: " + targetHeadingVect);
        SmartDashboard.putString("Target Heading Vect", targetHeadingVect.toString());

        // compute the target heading
        double targetHeading = robotFieldPos.getHeading() + Math.atan(targetHeadingVect.getDx() / targetHeadingVect.getDy());

        System.out.println("Target Heading: " + targetHeading);
        SmartDashboard.putString("Target Heading", targetHeadingVect.toString());

        // the target in the robot's reference frame
        Vector3D targetRobotFrameRobotOrigin = Vector3D.add(Constants.kCameraRelativeToRobotCenter, targetUpdate.getCameraVector().rotate(Constants.kCameraToRobotMatrix)); 

        //the target in a coordinate system alligned with the field, but centered at the robot
        Vector3D targetFieldFrameRobotOrigin = targetRobotFrameRobotOrigin.rotateZAxis(-robotFieldPos.getHeading());

        System.out.println("Robot Oriented Robot Origin: " + targetRobotFrameRobotOrigin);
        SmartDashboard.putString("Robot Oriented Robot Origin", targetRobotFrameRobotOrigin.toString());

        System.out.println("Field Oriented Robot Origin: " + targetFieldFrameRobotOrigin);
        SmartDashboard.putString("Field Oriented Robot Origin", targetFieldFrameRobotOrigin.toString());

        // rotate the vector such that its x component is pointing forward to match path pursuit's coordinate system
        // targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(Math.PI/2);
        // targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateYAxis(Math.PI);

        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(Math.PI/2);
        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateXAxis(Math.PI);

        //the target's location relative to the field in 2 dimentions
        Vector2D targetFieldLocation = new Vector2D(targetFieldFrameRobotOrigin.getDx() + robotFieldPos.getX(), targetFieldFrameRobotOrigin.getDy() + robotFieldPos.getY());

        System.out.println("Target Field Location: " + targetFieldLocation.toString());

        RobotPos latestFieldPos = Robot.estimator.getPos();

        SmartDashboard.putNumber("target heading", targetHeading);
        SmartDashboard.putNumber("robot heading", latestFieldPos.heading);
        SmartDashboard.putString("start point", latestFieldPos.position.toString());
        SmartDashboard.putString("end point", targetFieldLocation.toString());

        DubinsPath dubinsPath = DubinPathCalculator.getBestPath(latestFieldPos.position, latestFieldPos.getHeading(), targetFieldLocation.getPoint(), targetHeading);
        
        if (!dubinsPath.isValid()) {
            System.out.println("INVALID PATH");
            forceFinish = true;
        }
        System.out.println("planned path");
        SmartDashboard.putString("target status", "planned path");
        return dubinsPath.getPath();
    }
  
    @Override
    protected void execute() {
        if (forceFinish) {
            return;
        }

        RobotPos latestPos = Robot.estimator.getPos();
        latestPos.setVelocities(Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel());
        System.out.println(latestPos.getX() + ", " + latestPos.getY());
        RobotCmd cmd = path.update(latestPos);
        Robot.drivetrain.setLeftVel(cmd.getLeftVel());
        Robot.drivetrain.setRightVel(cmd.getRightVel());
    }
  
    @Override
    protected boolean isFinished() {
        return (path != null && path.isFinished()) || forceFinish || this.isTimedOut();
    }
  
    @Override
    protected void end() {
        SmartDashboard.putString("exec status", "END ");
    }
  
    @Override
    protected void interrupted() {
    }
}