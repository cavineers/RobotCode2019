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

import frc.robot.commands.FollowPath;

public class DriveVisionPath extends Command {

    Path path;

    public DriveVisionPath() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        this.setTimeout(60); // set the command timeout to 10 seconds
        path = null;
        
        //attempt to obtain the image data from the camera
        TargetUpdate targetUpdate = null; // data received from the camera
        targetUpdate = Robot.reflectiveTapeCamera.getUpdate();

        calculateValues(targetUpdate);
        

    }

    protected void calculateValues(TargetUpdate targetUpdate) {
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
        Vector3D vtcrs_rotated = vtccs.rotate(Constants.kMrscs);
        Vector3D vtrrs = Vector3D.add(vtcrs_rotated, Constants.kVrscs);

        //the target in a coordinate system alligned with the field, but centered at the robot
        Vector3D vtrfs = vtrrs.rotateZAxis(robotFieldPos.getHeading());

        Vector2D v2trfs = new Vector2D((vtrfs.getDx() - 8.0) + robotFieldPos.getX(), vtrfs.getDy() + robotFieldPos.getY());

        double targetHeadingAngle = robotFieldPos.getHeading() + Math.atan((vhtrs.getDy()/vhtrs.getDx()));       
        
        RobotPos latestFieldPos = Robot.estimator.getPos();

        //turn to point in front of target
        new TurnToAngle(calcAngle(latestFieldPos.getX(), latestFieldPos.getY(), v2trfs.getPoint().getX(), v2trfs.getPoint().getY()));

        latestFieldPos = Robot.estimator.getPos();

        //create and follow line to point in front of target
        //turn towards target
        //drive towards target




    }
  
    protected double calcAngle(double startX, double startY, double endX, double endY){
        double angle = 0;
        double xVal = Math.abs(startX- endX);
        double yVal = Math.abs(startY- endY);
        angle = Math.atan(yVal/xVal);
        return angle;
    }

    
    @Override
    protected void execute() {
       
        RobotPos latestPos = Robot.estimator.getPos();
        latestPos.setVelocities(Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel());
        RobotCmd cmd = path.update(latestPos);
        Robot.drivetrain.setLeftVel(cmd.getLeftVel());
        Robot.drivetrain.setRightVel(cmd.getRightVel());
    }
  
    @Override
    protected boolean isFinished() {
        return (path != null && path.isFinished()) || this.isTimedOut();
    }
  
    @Override
    protected void end() {
        SmartDashboard.putString("exec status", "END ");
    }
  
    @Override
    protected void interrupted() {
    }
}