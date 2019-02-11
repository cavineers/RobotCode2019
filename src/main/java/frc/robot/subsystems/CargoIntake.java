package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
public class CargoIntake extends Subsystem {
    // Moter
    public WPI_TalonSRX intakeMoter = new WPI_TalonSRX(RobotMap.intakeMoter);

    // DoubleSolenoid
    private DoubleSolenoid posSol;

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
        posSol = new DoubleSolenoid(RobotMap.PCM1, RobotMap.cargoIntake1, RobotMap.cargoIntake2);

        // Method 1
        intakeMoter.configPeakCurrentLimit(2, 10);
        intakeMoter.configPeakCurrentDuration(200, 10);
        intakeMoter.configContinuousCurrentLimit(1, 10);
        intakeMoter.enableCurrentLimit(true);
    }

    public void currentLimit() {
        // Method 2
        // if (intakeMoter.getOutputCurrent() > 1.0) { // If amps is creater than 10
        //     intakeMoter.set(0); // Stop the moter
        //     mstate = moterState.OFF; // Set the state to off
        // }
    }

    /**
     * Turn on the intake motors
     */
    public void on() {
        intakeMoter.set(Constants.kIntakeSpeed);
        mstate = moterState.ON;
        currentLimit(); // Run the currentLimit function
    }

    /**
     * Turn off the intake motors
     */
    public void off() {
        intakeMoter.set(0);
        mstate = moterState.OFF;
    }

    /**
     * Gets whether the intake motor is activated
     */
    public moterState getMoterState() {
        return mstate;
    }

    /**
     * Sets the position of the state to the desired state.
     * 
     * @param state the desired state of the cargo intake's arm
     */
    public void setPosition(positionState state) {
        if (state == positionState.UP) {
            posSol.set(Value.kForward);
        } else {
            posSol.set(Value.kReverse);
        }
    }
    
    /**
     * Gets the position of the intake arm (whether it is up or down)
     */
    public positionState getPosition() {
        if (posSol.get() == Value.kForward) {
            return positionState.UP;
        } else {
            return positionState.DOWN;
        }
    }
}