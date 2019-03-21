package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.commands.ChangeCargoIntakeState;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber;

public class ExtendGrabber extends CommandGroup {

    public ExtendGrabber() {
        requires(Robot.grabber);
        requires(Robot.elevator);
        requires(Robot.cargoIntake);
        addSequential(new ElevatorToLevel(ElevatorLevel.GROUND));
        addSequential(new ChangeGrabberState(GrabberPosition.EXTENDED, Grabber.MotorState.OFF));
    }

    @Override
    public void end() {
        System.out.println("extended finished:" + this.isFinished());
    }

    @Override
    protected void interrupted() {
        this.end(); //make sure that the end method is called even if the command is interrupted / canceled 
    }
}