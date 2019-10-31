package frc.robot.commands.auto;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.AutoPathHelper;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FollowPath;
import frc.robot.commands.HomeAll;
import frc.robot.commands.grabber.ChangeHatchGrabberState;
import frc.robot.subsystems.Grabber.HatchGrabberState;

public class DisableAutoOverride extends Command{
    
    public DisableAutoOverride() {
		this.setRunWhenDisabled(true);
	}
	@Override
	protected void initialize() {
		Robot.isAutoOverridden = false;
	}
	@Override
	protected boolean isFinished() {
		return true;
	}

}