package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Fork.forkState;

public class MoveFork extends Command {
    protected DoubleSolenoid forkSolCommand;
    protected forkState toForkState;

    public MoveFork(DoubleSolenoid forkSol, forkState state) {
        requires(Robot.fork);
        toForkState = state;
    }

    @Override
    public void initialize() {
        Robot.fork.setFork(toForkState);
    }

    public void setUp(boolean setup) {

    }

    protected boolean isFinished() {
        return Robot.fork.getState() == toForkState;
    }
}