package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.subsystems.CargoIntake.MotorState;
import frc.robot.subsystems.CargoIntake.PositionState;

public class ToggleCargoIntake extends ConditionalCommand {

    public ToggleCargoIntake() {
        //first argument runs if condition() returns true, second one if it returns false
        super(new ChangeCargoIntakeState(PositionState.UP, MotorState.OFF), new ChangeCargoIntakeState(PositionState.DOWN, MotorState.OFF));
    }

    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected boolean condition() {
        return Robot.cargoIntake.getPosition() == PositionState.DOWN;
    }

}