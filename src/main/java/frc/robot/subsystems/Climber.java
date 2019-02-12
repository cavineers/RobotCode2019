package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Climber extends Subsystem {
    private CANSparkMax m_motor;
    public LegState desiredState;
    public DoubleSolenoid armSol;

    double positionOffset = 0;

    public enum LegState {
        DEPLOYED,  //legs are down
        RETRACTED,  //legs are up
        INVALID // legs are somewhere in between
    }

    public enum ArmState {
        DEPLOYED, //arms are down
        RETRACTED //arms are within the frame perimeter
    }

    public Climber() {
        // initialize the motor
        m_motor = new CANSparkMax(RobotMap.climberMotor, MotorType.kBrushless);

        //zero the motor's encoder
        m_motor.getEncoder().getPosition();

        // set PID coefficients
        m_motor.getPIDController().setP(Constants.kPClimber);
        m_motor.getPIDController().setI(Constants.kIClimber);
        m_motor.getPIDController().setD(Constants.kDClimber);
        m_motor.getPIDController().setFF(Constants.kFClimber);
        m_motor.getPIDController().setIZone(Constants.kIZoneClimber);
        m_motor.getPIDController().setOutputRange(Constants.kClimberMinOutput, Constants.kClimberMaxOutput);

        // initialize the double solenoid in charge of controlling the climber arms
        armSol = new DoubleSolenoid(RobotMap.PCM2, RobotMap.climberArms1, RobotMap.climberArms2);
        this.retractArms();
        this.setPosition(0);
        this.retractLegs();
    }

    @Override
    public void initDefaultCommand() {
    }

    /**
     * Deploys the climber legs to the height needed for climbing
     */
    public void deployLegs() {
        this.setLegSetpoint((Constants.kClimberDeployPos + this.positionOffset) * Constants.kClimberRPI);
        this.desiredState = LegState.DEPLOYED;
    }
    
    /**
     * Retracts the climber's legs to their upright position
     */
    public void retractLegs() {
        this.setLegSetpoint(this.positionOffset);
        this.desiredState = LegState.RETRACTED;
    }

    /**
     * Sets the desired setpoint for the climber leg
     */
    private void setLegSetpoint(double setpointRotations) {
        this.m_motor.getPIDController().setReference(0, ControlType.kPosition);
    }

    /**
     * Sets the state of the legs (whether they are deployed or retracted)
     */
    public void setLegState(LegState state) {
        if (state == LegState.DEPLOYED) {
            this.deployLegs();
        } else {
            this.retractLegs();
        }
    }

    /**
     * Gets the desired state of the legs based on the last movement command
     */
    public LegState getDesiredLegState() {
        return this.desiredState;
    }

    /**
     * Gets the actual state of the legs based on the motor encoder
     */
    public LegState getLegState() {
        if (Math.abs(this.getLegPosition() - Constants.kClimberDeployPos) <= Constants.kClimberTolerance) {
            return LegState.DEPLOYED;
        } else if (Math.abs(this.getLegPosition() - 0) <= Constants.kClimberTolerance) {
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
    public double getLegPosition() {
        return (m_motor.getEncoder().getPosition() - this.positionOffset) / Constants.kClimberRPI;
    }

    /**
     * Sets the current position of the climber legs
     */
    public void setPosition(double position) {
        this.positionOffset = m_motor.getEncoder().getPosition() - position;
    }

    /**
     * Deploys the climber's stance arms
     */
    public void deployArms() {
        this.armSol.set(Value.kForward);
    }

    /**
     * Retracts the climber's stance arms
     */
    public void retractArms() {
        this.armSol.set(Value.kReverse);
    }

    /**
     * Toggles the state of the climber's stance arms
     */
    public void toggleArms() {
        if (this.getArmState() == ArmState.DEPLOYED) {
            this.retractArms();
        } else {
            this.deployArms();
        }
    }

    /**
     * Gets the current state of the climber arms
     */
    public ArmState getArmState() {
        if (this.armSol.get() == Value.kForward) {
            return ArmState.DEPLOYED;
        } else {
            return ArmState.RETRACTED;
        }
    }

}