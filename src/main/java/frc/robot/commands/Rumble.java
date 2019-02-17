package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class Rumble extends TimedCommand {
	public enum ControllerSide {
		RIGHT, LEFT, BOTH
	}
	public Rumble(double time, ControllerSide side) {
		super(time);
		if (side == ControllerSide.LEFT) {
			Robot.oi.getJoystick().setRumble(RumbleType.kLeftRumble, 1);
		}
		if (side == ControllerSide.RIGHT) {
			Robot.oi.getJoystick().setRumble(RumbleType.kRightRumble, 1);
		}
		if (side == ControllerSide.BOTH) {
			Robot.oi.getJoystick().setRumble(RumbleType.kLeftRumble, 1);
			Robot.oi.getJoystick().setRumble(RumbleType.kRightRumble, 1);
		}
	}
	
	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}
	protected void end() {
		Robot.oi.getJoystick().setRumble(RumbleType.kLeftRumble, 0);
		Robot.oi.getJoystick().setRumble(RumbleType.kRightRumble, 0);
    }
}
