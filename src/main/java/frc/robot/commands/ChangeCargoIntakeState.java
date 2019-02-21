package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Grabber.GrabberPosition;

public class ChangeCargoIntakeState extends Command {
    MotorState desiredMotorState;
    PositionState desiredPositionState;
    boolean forceFinish = false;

    public ChangeCargoIntakeState(PositionState desiredPositionState, MotorState desiredMotorState) {
        requires(Robot.cargoIntake);
        this.desiredPositionState = desiredPositionState;
        this.desiredMotorState = desiredMotorState;
    }
    
    @Override
    public void initialize() {
        Robot.cargoIntake.setMotorState(desiredMotorState);
        if (Robot.grabber.getState() == GrabberPosition.EXTENDED || Robot.grabber.getState() == GrabberPosition.HOMED) {
            Robot.cargoIntake.setState(desiredPositionState);
        }
    }

    @Override
    public void execute() {
    }

    protected boolean isFinished() {
        return Robot.cargoIntake.getPosition() == this.desiredPositionState;
    }
}