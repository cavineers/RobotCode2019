package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.MotorState;

public class MoveGrabberAndElevator extends CommandGroup {

    public MoveGrabberAndElevator(ElevatorLevel level, GrabberPosition grabberPos, MotorState motorState) {
        addParallel(new ElevatorToLevel(level));
        addParallel(new Command() {

            @Override
            protected void initialize() {
                System.out.println("move elevator motor state: " + grabberPos);
            }

            @Override
            protected boolean isFinished() {
                return true;
            }

        });
        addParallel(new ChangeGrabberState(grabberPos, motorState));
    }

}