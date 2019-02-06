package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    boolean didFinishInit = false;
    boolean didStartPathPursuit = false;
    Thread generatePath;
    Thread pursuePath;

    public TargetVisionTape() {
        requires(Robot.drivetrain);
        this.setTimeout(30); // set the command timeout to 10 seconds
    }

    @Override
    protected void initialize() {
        generatePath = new Thread() {
            public void run() {
                TargetUpdate targetUpdate = null; //data received from the camera

                while (targetUpdate == null && !forceFinish && !isTimedOut()) {
                    System.out.println("update: " + targetUpdate);
                    SmartDashboard.putString("target status", "getting target");
                    targetUpdate = Robot.reflectiveTapeCamera.getUpdate();
                }

                if (targetUpdate == null || forceFinish || isTimedOut()) {
                    SmartDashboard.putString("target status", "null 1: " + targetUpdate + ", " + forceFinish);
                    return; //catch the case where the command is interupted during init
                }
                System.out.println("Found Target");
                SmartDashboard.putString("target status", "found target");

                RobotPos robotFieldPos = Robot.estimator.getPositionAtTime(targetUpdate.getTimestamp()); // the position of the robot when the picture was taken (field coords)
        
                //convert the target heading vector to the robot's coordinate system
                Vector3D targetHeadingVect = targetUpdate.getHeadingVector();
                targetHeadingVect.rotate(Constants.kCameraToRobotMatrix);

                // compute the target heading
                double targetHeading = robotFieldPos.getHeading() + Math.atan(targetHeadingVect.getDx() / targetHeadingVect.getDy());

                // the target in the robot's reference frame
                Vector3D targetRobotFrameRobotOrigin = Vector3D.add(Constants.kCameraRelativeToRobotCenter, targetUpdate.getCameraVector().rotate(Constants.kCameraToRobotMatrix)); 

                //the target in a coordinate system alligned with the field, but centered at the robot
                Vector3D targetFieldFrameRobotOrigin = targetRobotFrameRobotOrigin.rotateZAxis(-robotFieldPos.getHeading());

                // rotate the vector such that its x component is pointing forward to match path pursuit's coordinate system
                targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(Math.PI/2);
                targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateYAxis(Math.PI);

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
                path = dubinsPath.getPath();
                System.out.println(path);
                didFinishInit = true;
                System.out.println("planned path");
                SmartDashboard.putString("target status", "planned path");
            }
        };
        generatePath.start();
    }
  
    @Override
    protected void execute() {
        int loopNum = 0;
        loopNum++;
        if (!didFinishInit) {
            SmartDashboard.putString("exec status", "no finish init " + loopNum);
            return;
        }
        if (didStartPathPursuit) {
            SmartDashboard.putString("exec status", "started path pursuit " + loopNum);
            return;
        }
        SmartDashboard.putString("exec status", "running " + loopNum);
        pursuePath = new Thread() {
            public void run() {
                while (!isFinished()) {
                    RobotPos latestPos = Robot.estimator.getPos();
                    latestPos.setVelocities(Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel());
                    
                    RobotCmd cmd = path.update(latestPos);
                    // System.out.println(latestPos.getX() + ", " + latestPos.getY());
                    
                    Robot.drivetrain.setLeftVel(cmd.getLeftVel());
                    Robot.drivetrain.setRightVel(cmd.getRightVel());
                    System.out.println("Driving Path");
                    SmartDashboard.putString("target status", "Driving Path");
                }
            }
        };
        pursuePath.start();
        didStartPathPursuit = true;
    }
  
    @Override
    protected boolean isFinished() {
        if ((path != null && path.isFinished()) || forceFinish || this.isTimedOut()) {
            System.out.println("ENDED");
            System.out.println(path);
            System.out.println(path.isFinished());
            System.out.println(forceFinish);
            System.out.println(this.isTimedOut());
        }
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