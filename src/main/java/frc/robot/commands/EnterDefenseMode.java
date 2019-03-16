package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;

public class EnterDefenseMode extends CommandGroup {
    
    public EnterDefenseMode() {
        addSequential(new ElevatorToLevel(ElevatorLevel.GROUND));
        addSequential(new ChangeGrabberState(GrabberPosition.START_POS));
        addSequential(new TimedCommand(0.5)); //stop the grabber from hitting the cargo intake
        addSequential(new ChangeCargoIntakeState(PositionState.UP, MotorState.OFF));
    }

}