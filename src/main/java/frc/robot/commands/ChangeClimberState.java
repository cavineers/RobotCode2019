package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Climber.LegState;
import frc.robot.OI;

public class ChangeClimberState extends Command {
    LegState desiredState;
    public ChangeClimberState(LegState desiredState) {
        requires(Robot.climber);
        requires(Robot.elevator);
        this.desiredState = desiredState;
        this.setInterruptible(false);
    }

    
    @Override
    public void initialize() {
        Robot.climber.setLegState(desiredState);
    }

    @Override
    public void execute() {
    }

    @Override
    protected void interrupted() {
        this.end(); //make sure that the end method is called even if the command is interrupted / canceled 
    }

    @Override
    protected boolean isFinished() {
        return Robot.climber.getLeg()==desiredState;
    }

    @Override
    public boolean isCanceled() {
        return super.isCanceled() || isFinished(); //stop the command from running when isFinished = false
    }

    @Override
    protected void end() {
        Robot.climber.getClimberMotor().set(0);
    }

    
}