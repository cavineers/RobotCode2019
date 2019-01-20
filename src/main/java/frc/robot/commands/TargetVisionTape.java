package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.RobotPos;
import frc.lib.dubinPath.DubinPathCalculator;
import frc.lib.dubinPath.DubinsPath;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.RobotCmd;
import frc.robot.Robot;

public class TargetVisionTape extends Command {

    Path path;

    boolean forceFinish = false;

    public TargetVisionTape() {
        requires(Robot.drivetrain);
    }

    @Override
    protected void initialize() {
        //TODO: implement after camera coms is finished
        DubinsPath dubinsPath = DubinPathCalculator.getBestPath(Robot.estimator.getPos().position, Robot.estimator.getHeading(), null, 0);
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