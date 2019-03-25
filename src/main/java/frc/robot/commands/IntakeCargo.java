package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.commands.grabber.ChangeHatchGrabberState;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.HatchGrabberState;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.HatchScoop.HatchScoopState;

public class IntakeCargo extends CommandGroup {
    boolean wasInterrupted = false;

    public IntakeCargo() {
        addSequential(new ChangeCargoIntakeState(PositionState.DOWN, MotorState.ON));
        addSequential(new MoveGrabberAndElevator(ElevatorLevel.GROUND, Grabber.GrabberPosition.RETRACTED, Grabber.MotorState.INTAKE_BALL));
    }
    

    @Override
    protected void initialize() {
        wasInterrupted = false;
        
        if (Robot.grabber.hasHatch()) {
            new Rumble(0.25, ControllerSide.BOTH).start();
            new ChangeHatchGrabberState(HatchGrabberState.INTAKING).start();
            new ChangeCargoIntakeState(PositionState.DOWN, MotorState.OFF).start();
        } 
        super.initialize();


    }

   
    @Override
    protected void end() {
        new ChangeGrabberState(Grabber.MotorState.OFF).start();
        new ChangeCargoIntakeState(PositionState.DOWN, MotorState.OFF).start();
        if (!wasInterrupted && Robot.grabber.hasCargo()) {
            //only change the hatch grabber state if the robot just picked up a ball
            new ChangeHatchGrabberState(HatchGrabberState.HOLDING).start();
        }
    }

    @Override
    protected void interrupted() {
        super.interrupted();
        
        wasInterrupted = true;
    }

    @Override
    protected boolean isFinished() {
        return Robot.grabber.hasCargo() || Robot.grabber.hasHatch();
    }

}