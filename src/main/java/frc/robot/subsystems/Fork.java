package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Fork extends Subsystem {
    private DoubleSolenoid forkSol;

    @Override
    public void initDefaultCommand() {

    }

    public enum forkState {
        UP, DOWN
    }

    public Fork() {
        forkSol = new DoubleSolenoid(RobotMap.PCM, 0);
    }

    public void toggleFork() {
        if (forkSol.get() == Value.kForward) {
            forkSol.set(Value.kReverse);
        } else {
            forkSol.set(Value.kForward);
        }
    }

    public void setFork(forkState state) {
        if (state == forkState.UP) {
            forkSol.set(Value.kReverse);
        } else {
            forkSol.set(Value.kForward);
        }
    }

    public forkState getState() {
        if (forkSol.get() == Value.kForward) {
            return forkState.DOWN;
        } else {
            return forkState.UP;
        }
    }
}
