package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.HatchIntake.forkState;

public class MoveFork extends Command {
    protected DoubleSolenoid forkSolCommand;
    protected forkState toForkState;

    public MoveFork(DoubleSolenoid forkSol, forkState state) {
        requires(Robot.hatchIntake);
        toForkState = state;
    }

    @Override
    public void initialize() {
        Robot.hatchIntake.setFork(toForkState);
    }

    public void setUp(boolean setup) {

    }

    protected boolean isFinished() {
        return Robot.hatchIntake.getState() == toForkState;
    }
}