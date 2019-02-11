package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchScoop extends Subsystem {
    private DoubleSolenoid hatchSol;

    @Override
    public void initDefaultCommand() {
    }

    public enum HatchScoopState {
        UP, DOWN
    }

    public HatchScoop() {
        hatchSol = new DoubleSolenoid(RobotMap.PCM1, RobotMap.hatchScoop1, RobotMap.hatchScoop2);
    }

    public void toggleFork() {
        if (hatchSol.get() == Value.kForward) {
            hatchSol.set(Value.kReverse);
        } else {
            hatchSol.set(Value.kForward);
        }
    }

    public void setState(HatchScoopState state) {
        if (state == HatchScoopState.UP) {
            hatchSol.set(Value.kReverse);
        } else {
            hatchSol.set(Value.kForward);
        }
    }

    public HatchScoopState getState() {
        if (hatchSol.get() == Value.kForward) {
            return HatchScoopState.DOWN;
        } else {
            return HatchScoopState.UP;
        }
    }
}
