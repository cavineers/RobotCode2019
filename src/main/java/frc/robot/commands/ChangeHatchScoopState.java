package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.HatchScoop.HatchScoopState;

public class ChangeHatchScoopState extends Command {
    protected HatchScoopState desiredState;

    public ChangeHatchScoopState(DoubleSolenoid hatchSol, HatchScoopState state) {
        requires(Robot.hatchScoop);
        desiredState = state;
    }

    @Override
    public void initialize() {
        Robot.hatchScoop.setState(desiredState);
    }

    public void setUp(boolean setup) {
    }

    protected boolean isFinished() {
        return Robot.hatchScoop.getState() == desiredState;
    }
}