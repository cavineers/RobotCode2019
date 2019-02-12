package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.CargoIntake.moterState;
import frc.robot.Robot;

public class ToggleCargoIntake extends Command {
    public moterState toState;

    public ToggleCargoIntake() {
        requires(Robot.cargoIntake);
    }

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

    protected boolean isFinished() {
        return Robot.cargoIntake.mstate == toState;
    }
}