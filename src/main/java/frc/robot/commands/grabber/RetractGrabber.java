package frc.robot.commands.grabber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.commands.ChangeCargoIntakeState;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.subsystems.Grabber;

public class RetractGrabber extends CommandGroup {

    public RetractGrabber() {
        addSequential(new ChangeCargoIntakeState(PositionState.DOWN, MotorState.OFF));
        addSequential(new ElevatorToLevel(ElevatorLevel.GROUND));
        addSequential(new ChangeGrabberState(GrabberPosition.RETRACTED, Grabber.MotorState.OFF));
    }
    
    @Override
    protected void initialize() {
        if (isFinished()) {
            new Rumble(0.25, ControllerSide.BOTH).start();
        }
    }

    @Override
    protected void interrupted() {
        this.end(); //make sure that the end method is called even if the command is interrupted / canceled 
    }
}