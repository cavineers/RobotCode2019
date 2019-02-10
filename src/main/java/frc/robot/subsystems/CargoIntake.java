package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;

public class CargoIntake extends Subsystem {
    // Moter
    public WPI_TalonSRX intakeMoter = new WPI_TalonSRX(RobotMap.intakeMoter);

    // DoubleSolenoid
    private Solenoid posSol;

    // States
    public enum moterState {
        ON, OFF
    }

    public enum positionState {
        UP, DOWN
    }

    // State variables
    public moterState mstate = moterState.ON;

    @Override
    public void initDefaultCommand() {

    }

    public CargoIntake() {
        posSol = new Solenoid(RobotMap.PCM1, 0);

        // Method 1
        intakeMoter.configPeakCurrentLimit(10); // Set max amps to 10
        intakeMoter.configPeakCurrentDuration(500); // Enable after 500 miliseconds
        intakeMoter.enableCurrentLimit(true); // Enable
    }

    public void on() {
        intakeMoter.set(Constants.kIntakeSpeed);
        mstate = moterState.ON;
        currentLimit(); // Run the currentLimit function
    }

    public void currentLimit() {
        // Method 2
        if (intakeMoter.getOutputCurrent() > 10.0) { // If amps is creater than 10
            intakeMoter.set(0); // Stop the moter
            mstate = moterState.OFF; // Set the state to off
        }
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