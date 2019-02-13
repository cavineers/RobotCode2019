package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
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
        requires(Robot.hatchScoop);
        addSequential(new ChangeHatchScoopState(HatchScoopState.UP));
        addSequential(new ElevatorToLevel(ElevatorLevel.CARGO_INTAKE));
        addSequential(new ChangeCargoIntakeState(PositionState.DOWN, MotorState.ON));
        addSequential(new ChangeGrabberState(Grabber.GrabberState.RETRACTED, Grabber.MotorState.INTAKE_BALL));
    }
   
    @Override
    protected boolean isFinished() {
        return super.isFinished() || OI.a_button.get() || Robot.grabber.hasCargo(); //TODO: finish if there is a ball already in the grabber
    }

    @Override
    protected void end() {
        Command shutoffIntake = new ChangeCargoIntakeState(CargoIntake.PositionState.UP, CargoIntake.MotorState.OFF);
        shutoffIntake.start();
        shutoffIntake.close();
    }
  


}