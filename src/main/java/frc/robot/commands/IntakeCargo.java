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
    boolean forceFinish;
    public IntakeCargo() {
        addSequential(new ChangeCargoIntakeState(PositionState.DOWN, MotorState.ON));
        addSequential(new MoveGrabberAndElevator(ElevatorLevel.GROUND, Grabber.GrabberPosition.RETRACTED, Grabber.MotorState.INTAKE_BALL));
    }

    @Override
    protected void initialize() {
        super.initialize();
        if (Robot.grabber.hasHatch()) {
            new Rumble(0.25, ControllerSide.BOTH).start();
        }
    }
   
    @Override
    protected void end() {
        new ChangeCargoIntakeState(CargoIntake.PositionState.DOWN, CargoIntake.MotorState.OFF).start();
        new ChangeGrabberState(Grabber.MotorState.OFF).start();
    }

    @Override
    protected boolean isFinished() {
        return Robot.grabber.hasCargo() || Robot.grabber.hasHatch();
    }

}