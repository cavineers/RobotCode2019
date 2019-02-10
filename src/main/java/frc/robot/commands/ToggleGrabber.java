package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake.moterState;
import frc.robot.Robot;

public class ToggleGrabber extends Command {
    public moterState toState;

    @Override
    public void initialize() {
        if (Robot.cargoIntake.mstate == moterState.OFF) {
            Robot.cargoIntake.on();
            toState = moterState.ON;
        } else {
            Robot.cargoIntake.off();
            toState = moterState.OFF;
        }
    }

    public ToggleGrabber() {
        requires(Robot.cargoIntake);
    }

    protected boolean isFinished() {
        return Robot.cargoIntake.mstate == toState;
    }
}