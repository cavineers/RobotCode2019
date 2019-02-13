package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Climber.LegState;

public class ChangeCargoIntakeState extends Command {
    MotorState desiredMotorState;
    PositionState desiredPositionState;

    public ChangeCargoIntakeState(MotorState desiredMotorState, PositionState desiredPositionState) {
        requires(Robot.cargoIntake);
        this.desiredMotorState = desiredMotorState;
        this.desiredPositionState = desiredPositionState;
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