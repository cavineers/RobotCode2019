package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CoordinateFrameHelper;
import frc.lib.MathHelper;
import frc.lib.RobotPos;
import frc.lib.TargetUpdate;
import frc.lib.Vector2D;
import frc.lib.Vector3D;
import frc.lib.dubinPath.DubinPathCalculator;
import frc.lib.dubinPath.DubinsPath;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.RobotCmd;
import frc.robot.Robot;
import frc.lib.RobotPos;
import frc.lib.pathPursuit.Point;
import frc.robot.Constants;

public class TargetVisionTape extends Command {

    Path path;

    boolean forceFinish = false;

    public TargetVisionTape() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        //TODO: test

        TargetUpdate targetUpdate = Robot.reflectiveTapeCamera.getUpdate(); //position of the robot on the target's field coords

        RobotPos robotFieldPos = Robot.estimator.getPositionAtTime(targetUpdate.getTimestamp()); // the position of the robot when the picture was taken (field coords)

        double angleA = Math.atan(targetUpdate.getTargetVector().getDx() / targetUpdate.getTargetVector().getDz());
        double angleB = Math.atan(targetUpdate.getCameraVector().getDx() / targetUpdate.getCameraVector().getDz());

        double targetHeading = robotFieldPos.getHeading() + angleA - angleB; //heading of the target

        // the target in the robot's reference frame
        Vector3D targetRobotFrameRobotOrigin = Vector3D.add(Constants.kCameraRelativeToRobotCenter, targetUpdate.getCameraVector().rotateXAxis(Math.PI/2)); 

        //the target in a coordinate system alligned with the field, but centered at the robot
        Vector3D targetFieldFrameRobotOrigin = targetRobotFrameRobotOrigin.rotateZAxis(-robotFieldPos.getHeading());

        // rotate the vector such that its x component is pointing forward to match path pursuit's coordinate system
        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(-Math.PI/2);
        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateYAxis(Math.PI);

        //the target's location relative to the field in 2 dimentions
        Vector2D targetFieldLocation = new Vector2D(targetFieldFrameRobotOrigin.getDx() + robotFieldPos.getX(), targetFieldFrameRobotOrigin.getDy() + robotFieldPos.getY());

        RobotPos latestFieldPos = Robot.estimator.getPos();

        DubinsPath dubinsPath = DubinPathCalculator.getBestPath(latestFieldPos.position, latestFieldPos.getHeading(), targetFieldLocation.getPoint(), targetHeading);
        
        if (!dubinsPath.isValid()) {
            forceFinish = true;
        }
        this.path = dubinsPath.getPath();
        
    }
  
    @Override
    protected void execute() {
        RobotPos currentPos = Robot.estimator.getPos();
        currentPos.setVelocities(Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel());
        
        RobotCmd cmd = path.update(currentPos);

        Robot.drivetrain.setLeftVel(cmd.getLeftVel());
        Robot.drivetrain.setRightVel(cmd.getRightVel());
    }
  
    @Override
    protected boolean isFinished() {
        return path.isFinished() || forceFinish;
    }
  
    @Override
    protected void end() {

    }
  
    @Override
    protected void interrupted() {
    }
}