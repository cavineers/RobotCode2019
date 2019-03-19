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

public class VisionMath extends Command {

    Path path;
    boolean forceFinish;

    public VisionMath() {
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
        
        //targetHeadingVect
        Vector3D vhtcs = targetUpdate.getVhtcs();
        
        //targetVect
        Vector3D vctts = targetUpdate.getVctts();

        //cameraVect
        Vector3D vtccs = targetUpdate.getVtccs();

        //transform Target Heading Vector into robot coordinate system
        Vector3D vhtrs = vhtcs.rotate(Constants.kMrscs);

        //transform Camera Vector into robot coordinate system
        Vector3D vtccs_rotated = vtccs.rotate(Constants.kMrscs);
        Vector3D vtcrs = Vector3D.add(vtccs_rotated, Constants.kVrscs);

        //transform Target Vector into robot coordinate system
        Vector3D vctrs = vctts.rotate(Constants.kMtscs);
        
        //the target in a coordinate system alligned with the field, but centered at the robot
        Vector3D targetFieldFrameRobotOrigin = vctrs.rotateZAxis(-robotFieldPos.getHeading());
        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(Math.PI/2);
        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateXAxis(Math.PI);
        Vector2D targetFieldLocation = new Vector2D(targetFieldFrameRobotOrigin.getDx() + robotFieldPos.getX(), targetFieldFrameRobotOrigin.getDy() + robotFieldPos.getY());

        double targetHeadingAngle = robotFieldPos.getHeading() + Math.atan((vhtrs.getDy()/vhtrs.getDx()));       
      
        RobotPos latestFieldPos = Robot.estimator.getPos();

        DubinsPath dubinsPath = DubinPathCalculator.getBestPath(latestFieldPos.position, latestFieldPos.getHeading(), targetFieldLocation.getPoint(), targetHeadingAngle);
       
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