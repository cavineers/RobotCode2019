package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber.GrabberState;
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
        addSequential(new ChangeCargoIntakeState(MotorState.ON, PositionState.DOWN));
        addSequential(new ChangeGrabberState(GrabberState.RETRACTED));
    }
   
    @Override
    protected boolean isFinished() {
        return super.isFinished() || OI.a_button.get(); //TODO: finish if there is a ball already in the grabber
    }
  


}