package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber.Position;
import frc.robot.Robot;

public class LowerClimber extends Command {
    @Override
    public void initialize() {
        Robot.climber.deploy();
    }

    public LowerClimber() {
        requires(Robot.climber);
    }

    protected boolean isFinished() {
        return Robot.climber.position == Position.DEPLOYED;
    }
}