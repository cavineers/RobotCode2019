package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.commands.grabber.ChangeHatchGrabberState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.HatchGrabberState;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;

public class EnterDefenseMode extends CommandGroup {
    
    public EnterDefenseMode() {
        addSequential(new MoveGrabberAndElevator(ElevatorLevel.GROUND, GrabberPosition.START_POS, frc.robot.subsystems.Grabber.MotorState.OFF));
        addSequential(new ChangeCargoIntakeState(PositionState.UP, MotorState.OFF));
        addSequential(new ChangeHatchGrabberState(HatchGrabberState.HOLDING));
    }

}