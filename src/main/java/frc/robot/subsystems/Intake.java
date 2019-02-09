package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
// TODO: Switch to DoubleSolenoid
public class Intake extends Subsystem {
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

    public Intake() {
        posSol = new Solenoid(RobotMap.PCM, 0);

        // Method 1
        intakeMoter.configPeakCurrentLimit(2, 10);
        intakeMoter.configPeakCurrentDuration(200, 10);
        intakeMoter.configContinuousCurrentLimit(1, 10);
        intakeMoter.enableCurrentLimit(true);
    }

    public void on() {
        intakeMoter.set(Constants.kIntakeSpeed);
        mstate = moterState.ON;
        currentLimit(); // Run the currentLimit function
    }

    public void currentLimit() {
        // Method 2
        // if (intakeMoter.getOutputCurrent() > 1.0) { // If amps is creater than 10
        //     intakeMoter.set(0); // Stop the moter
        //     mstate = moterState.OFF; // Set the state to off
        // }
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