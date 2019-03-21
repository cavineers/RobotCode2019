package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.CargoIntake;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.HatchScoop.HatchScoopState;

public class IntakeCargo extends CommandGroup {
    boolean forceFinish;
    public IntakeCargo() {
        requires(Robot.grabber);
        requires(Robot.elevator);
        requires(Robot.cargoIntake);
        addSequential(new ChangeCargoIntakeState(PositionState.DOWN, MotorState.ON));
        addSequential(new ElevatorToLevel(ElevatorLevel.GROUND));
        // addSequential(new TimedCommand(0.5));
        addSequential(new ChangeGrabberState(Grabber.GrabberPosition.RETRACTED, Grabber.MotorState.INTAKE_BALL));
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

    @Override
    public boolean isCanceled() {
        return super.isCanceled() || isFinished(); //stop the command from running when isFinished = false
    }

    @Override
    protected void interrupted() {
        this.end(); //make sure that the end method is called even if the command is interrupted / canceled 
    }
  


}