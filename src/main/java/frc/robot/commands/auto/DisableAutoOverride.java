package frc.robot.commands.auto;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

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