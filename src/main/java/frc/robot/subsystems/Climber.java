package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends Subsystem {
    private CANSparkMax m_motor;
    public LegState desiredState;

    public enum LegState {
        DEPLOYED, RETRACTED, INVALID
    }

    public Climber() {
        m_motor = new CANSparkMax(RobotMap.climberMotor, MotorType.kBrushless); // Create the device
        m_motor.getEncoder().setPosition(0);

        // set PID coefficients
        m_motor.getPIDController().setP(Constants.kPClimber);
        m_motor.getPIDController().setI(Constants.kIClimber);
        m_motor.getPIDController().setD(Constants.kDClimber);
        m_motor.getPIDController().setFF(Constants.kFClimber);
        m_motor.getPIDController().setIZone(Constants.kIZoneClimber);
        m_motor.getPIDController().setOutputRange(Constants.kClimberMinOutput, Constants.kClimberMaxOutput);
    }

    @Override
    public void initDefaultCommand() {
    }

    /**
     * Deploys the climber legs to the height needed for climbing
     */
    public void deploy() {
        this.setSetpoint(Constants.kClimberDeployPos * Constants.kClimberRPI);
        this.desiredState = LegState.DEPLOYED;
    }
    
    /**
     * Retracts the climber's legs to their upright position
     */
    public void retract() {
        this.setSetpoint(0);
        this.desiredState = LegState.RETRACTED;
    }

    private void setSetpoint(double setpointRotations) {
        this.m_motor.getPIDController().setReference(0, ControlType.kPosition);
    }

    /**
     * Sets the state of the legs (whether they are deployed or retracted)
     */
    public void setState(LegState state) {
        if (state == LegState.DEPLOYED) {
            this.deploy();
        } else {
            this.retract();
        }
    }

    /**
     * Gets the desired state of the legs based on the last movement command
     */
    public LegState getDesiredState() {
        return this.desiredState;
    }

    /**
     * Gets the actual state of the legs based on the motor encoder
     */
    public LegState getState() {
        if (Math.abs(this.getPosition() - Constants.kClimberDeployPos) <= Constants.kClimberTolerance) {
            return LegState.DEPLOYED;
        } else if (Math.abs(this.getPosition() - 0) <= Constants.kClimberTolerance) {
            return LegState.RETRACTED;
        } else {
            return LegState.INVALID;
        }
    }

    /**
     * Gets the position of the legs below the ground in inches
     * 
     * @return position of the legs below the ground in inches
     */
    public double getPosition() {
        return m_motor.getEncoder().getPosition() / Constants.kClimberRPI;
    }
}