package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Elevator.ElevatorLevel;

public class ElevatorToLevel extends Command {

    public ElevatorLevel desiredLevel;

    public ElevatorToLevel(ElevatorLevel desiredLevel) {
        this.desiredLevel = desiredLevel;
    }

    @Override
    protected void initialize() {
        requires(Robot.elevator);
        Robot.elevator.moveElevator(this.desiredLevel);
    }

    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
    }

    @Override
    protected boolean isFinished() {
        return Robot.elevator.getLevel() == desiredLevel;
    }
}