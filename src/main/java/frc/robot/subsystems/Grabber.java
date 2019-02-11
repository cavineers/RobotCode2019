package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class Grabber extends Subsystem {

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward

    DoubleSolenoid grabberSol; // double solenoid for control of both the 
                               // ball's endstop and the hatch grabber

    public Grabber() {
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        grabberSol = new DoubleSolenoid(RobotMap.grabber1, RobotMap.grabber2);
    }

    @Override
    protected void initDefaultCommand() {
    }

}
