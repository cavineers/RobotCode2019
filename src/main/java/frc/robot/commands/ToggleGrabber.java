package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Intake.moterState;
import frc.robot.Robot;

public class ToggleGrabber extends Command {
    public moterState toState;

    @Override
    public void initialize() {
        if (Robot.intake.mstate == moterState.OFF) {
            Robot.intake.on();
            toState = moterState.ON;
        } else {
            Robot.intake.off();
            toState = moterState.OFF;
        }
    }

    public ToggleGrabber() {
        requires(Robot.intake);
    }

    protected boolean isFinished() {
        return Robot.intake.mstate == toState;
    }
}