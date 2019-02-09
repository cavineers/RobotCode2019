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
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    private double startingPosition;

    @Override
    public void initDefaultCommand() {

    }

    public Climber() {
        m_motor = new CANSparkMax(RobotMap.climberMotor, MotorType.kBrushless); // Create the device
        m_pidController = m_motor.getPIDController();
        m_encoder = m_motor.getEncoder();
        startingPosition = m_encoder.getPosition();

        // set PID coefficients
        m_pidController.setP(Constants.kPClimber);
        m_pidController.setI(Constants.kIClimber);
        m_pidController.setD(Constants.kDClimber);
        m_pidController.setFF(Constants.kFClimber);

        m_pidController.setIZone(Constants.kIZoneClimber);
        m_pidController.setOutputRange(Constants.kClimberMinOutput, Constants.kClimberMaxOutput);
    }

    public void deploy() {
        m_pidController.setReference(startingPosition + (Constants.kClimberLength * Constants.kClimberPPI),
                ControlType.kPosition);
    }

    public void retract() {
        m_pidController.setReference(startingPosition, ControlType.kPosition);
    }

    public double getLength() {
        return (m_encoder.getPosition() - startingPosition) / Constants.kClimberPPI;
    }

    public double getStartingLength() {
        return startingPosition / Constants.kClimberPPI;
    }
}