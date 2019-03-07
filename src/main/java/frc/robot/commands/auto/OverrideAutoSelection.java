package frc.robot.commands.auto;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class OverrideAutoSelection extends Command{
	public OverrideAutoSelection() {
		this.setRunWhenDisabled(true);
	}
	@Override
	protected void initialize() {
		Robot.isAutoOverridden = true;
        Robot.overriddenPathTarget = Robot.targetChooser.getSelected();
        Robot.overriddenStartPos = Robot.posChooser.getSelected();
	}
	@Override
	protected boolean isFinished() {
		return true;
	}

}