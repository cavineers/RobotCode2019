package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber.LegState;

public class ChangeClimberState extends Command {
    LegState desiredState;
    public ChangeClimberState(LegState desiredState) {
        requires(Robot.climber);
        this.desiredState = desiredState;
    }
    
    @Override
    public void initialize() {
        Robot.climber.setState(desiredState);
    }

    @Override
    public void execute() {
    }

    protected boolean isFinished() {
        return true;
    }
}