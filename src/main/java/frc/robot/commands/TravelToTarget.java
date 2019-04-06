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

public class TravelToTarget extends Command {
    TargetUpdate targetUpdate = null; 
    Path pathA;
    Path pathB;

    public TravelToTarget() {
        requires(Robot.drivetrain);
        
    }

    @Override
    protected void initialize() {
        this.setTimeout(60); 
        if(Robot.reflectiveTapeCamera.getUpdate() == null){
            end();
            return;
        }

        targetUpdate = Robot.reflectiveTapeCamera.getUpdate();
       
    }
  
    @Override
    protected void execute() {
       
    }
  
    @Override
    protected boolean isFinished() {
    }
  
    @Override
    protected void end() {
    }
  
    @Override
    protected void interrupted() {
    }
}