package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.CoordinateFrameHelper;
import frc.lib.MathHelper;
import frc.lib.RobotPos;
import frc.lib.TargetPos;
import frc.lib.Vector;
import frc.lib.dubinPath.DubinPathCalculator;
import frc.lib.dubinPath.DubinsPath;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.RobotCmd;
import frc.robot.Robot;
import frc.lib.RobotPos;
import frc.lib.pathPursuit.Point;

public class TargetVisionTape extends Command {

    Path path;

    boolean forceFinish = false;

    public TargetVisionTape() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        //TODO: test

        TargetPos robotPosTarget = Robot.reflectiveTapeCamera.getUpdate(); //position of the robot on the target's field coords

        RobotPos initialPosField = Robot.estimator.getPositionAtTime(robotPosTarget.getTimestamp()); // the position of the robot when the picture was taken (field coords)

        // displacement since the picture was taken (field coords)
        RobotPos movementField = Robot.estimator.getMovementSinceTime(robotPosTarget.getTimestamp()); 

        // the position of the robot on the field's coordinate frame when the picture was taken
        Point initialRobotPosField =  CoordinateFrameHelper.getFieldRobotPos(initialPosField.getHeading(), robotPosTarget.getAngle(), robotPosTarget.getPosition());

        // the angle of the target relative to the field
        double targetAngle = initialPosField.getHeading() - robotPosTarget.getAngle();

        RobotPos robotPosField = new RobotPos(movementField.getX() + initialRobotPosField.getX(), movementField.getY() + initialRobotPosField.getY(), Robot.getAngleRad(), 0, 0);
        
        DubinsPath dubinsPath = DubinPathCalculator.getBestPath(robotPosField.position, robotPosField.getHeading(), new Point(0,0), targetAngle);
        
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
        return false || forceFinish;
    }
  
    @Override
    protected void end() {

    }
  
    @Override
    protected void interrupted() {
    }
}