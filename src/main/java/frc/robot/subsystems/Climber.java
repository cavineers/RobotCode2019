package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;

public class Climber extends Subsystem {
    private CANSparkMax m_motor;
    private static final int deviceID = 1;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double startingPosition;

    @Override
    public void initDefaultCommand() {

    }

    public Climber() {
        m_motor = new CANSparkMax(deviceID, MotorType.kBrushless); // Create the device
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        startingPosition = m_encoder.getPosition();

        // PID coefficients
        kP = 0.1;
        kI = 1e-4;
        kD = 1;
        kIz = 0;
        kFF = 0;
        kMaxOutput = 1;
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public void deploy() {
        m_pidController.setReference(startingPosition + (Constants.climberLength * Constants.climberPPI),
                ControlType.kPosition);
    }

    public void retract() {
        m_pidController.setReference(startingPosition, ControlType.kPosition);
    }

    public double getLength() {
        return (m_encoder.getPosition() - startingPosition) / Constants.climberPPI;
    }

    public double getStartingLength() {
        return startingPosition / Constants.climberPPI;
    }
}