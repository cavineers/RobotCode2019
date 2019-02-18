package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;

public class ChangeCargoIntakeState extends Command {
    MotorState desiredMotorState;
    PositionState desiredPositionState;

    public ChangeCargoIntakeState(PositionState desiredPositionState, MotorState desiredMotorState) {
        requires(Robot.cargoIntake);
        this.desiredPositionState = desiredPositionState;
        this.desiredMotorState = desiredMotorState;
    }
    
    @Override
    public void initialize() {
        // Robot.cargoIntake.setMo
        Robot.cargoIntake.setPosition(desiredPositionState);
        Robot.cargoIntake.setMotorState(desiredMotorState);
    }

    @Override
    public void execute() {
    }

    protected boolean isFinished() {
        return true;
    }
}