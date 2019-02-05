package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;


public class Intake extends Subsystem {
    // Moter
    public WPI_TalonSRX intakeMoter = new WPI_TalonSRX(RobotMap.intakeMoter);

    // DoubleSolenoid
    private Solenoid posSol;

    // States
    public enum moterState {
        ON,
        OFF
    }

    public enum positionState {
        UP,
        DOWN
    }

    // State variables
    public moterState mstate = moterState.ON;

    @Override
    public void initDefaultCommand() {

    }

    public Intake() {
        posSol = new Solenoid(RobotMap.PCM, 0);		
    }

    public void on() {
        intakeMoter.set(Constants.intakeSpeed);
        mstate = moterState.ON;
    }

    public void off() {
        intakeMoter.set(0);
        mstate = moterState.OFF;
    }

    public void up() {
        posSol.set(true);
    }

    public void down() {
        posSol.set(false);
    }

    public moterState getMoterState() {
        return mstate;
    }

    public positionState getPositionState() {
        if (posSol.get() == true) {
            return positionState.UP;
        } else {
            return positionState.DOWN;
        }
    }
}