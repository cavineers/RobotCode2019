package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.commands.ChangeCargoIntakeState;
import frc.robot.commands.MoveGrabberAndElevator;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Grabber;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;

public class ExtendGrabber extends CommandGroup {

    public ExtendGrabber() {
        addSequential(new MoveGrabberAndElevator(ElevatorLevel.GROUND, GrabberPosition.EXTENDED, Grabber.MotorState.OFF));
        addSequential(new ChangeCargoIntakeState(Robot.cargoIntake.getPosition(), CargoIntake.MotorState.OFF));
    }
}