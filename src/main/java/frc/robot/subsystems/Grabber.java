package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Constants;
import frc.robot.Robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;


public class Grabber extends Subsystem {
    public enum GrabberState {
        EXTENDED,
        RETRACTED,
        START_POS,
        HOMED,
        UNKNOWN
    }

    public enum MotorState {
        OFF,
        INTAKE_BALL,
        EJECT_BALL
    }
    public GrabberState grabberState;

    CANSparkMax armMotor; // motor responcible moving the arm forward and backward
    WPI_TalonSRX ballMotor; // motor responcible for minipulating the ball in the grabber
    private PIDController pidPos;

    DoubleSolenoid grabberSol; // double solenoid for control of both the 
                               // ball's endstop and the hatch grabber

    double positionOffset; // a correction value used to offset position from homing

    DigitalInput cargoLimitSwitch = new DigitalInput(Constants.kGrabberCargoLimitSwitch);
    DigitalInput hatchLimitSwitch = new DigitalInput(Constants.kGrabberHatchLimitSwitch);

    LinearDigitalFilter averageCurrent;

    PIDSource currentSource = new PIDSource() {
        PIDSourceType vel_sourceType = PIDSourceType.kDisplacement;

        @Override
        public double pidGet() {
            return Robot.grabber.getGrabberMotor().getOutputCurrent();
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            vel_sourceType = pidSource;
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return vel_sourceType;
        }

    };

    public Grabber() {
        grabberState = GrabberState.UNKNOWN;
        armMotor = new CANSparkMax(RobotMap.armMotor, MotorType.kBrushless);
        grabberSol = new DoubleSolenoid(RobotMap.grabber1, RobotMap.grabber2);
        positionOffset = 0;
       
        this.getGrabberMotor().getPIDController().setP(Constants.kGrabberVelP);
        this.getGrabberMotor().getPIDController().setI(Constants.kGrabberVelI);
        this.getGrabberMotor().getPIDController().setD(Constants.kGrabberVelD);
        this.getGrabberMotor().getPIDController().setFF(Constants.kGrabberVelF);
        this.getGrabberMotor().getPIDController().setOutputRange(-1, 1);
        this.getGrabberMotor().setIdleMode(IdleMode.kBrake);

        

        pidPos = new PIDController(Constants.kGrabberPosP, Constants.kGrabberPosI, Constants.kGrabberPosD, Constants.kGrabberPosF, new PIDSource() {
            PIDSourceType vel_sourceType = PIDSourceType.kDisplacement;

            @Override
            public double pidGet() {
                return armMotor.getEncoder().getPosition();
            }

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                vel_sourceType = pidSource;
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return vel_sourceType;
            }

        }, new PIDOutput() {
            @Override
            public void pidWrite(double d) {
                Robot.grabber.getGrabberMotor().getPIDController().setReference(d, ControlType.kVelocity);
            }
        }, Constants.kGrabberPIDPosPeriod);

        pidPos.setInputRange(Constants.kMinGrabberPos, Constants.kMaxGrabberPos); 
        pidPos.setOutputRange(-Constants.kGrabberMaxSpeed, Constants.kGrabberMaxSpeed);
        pidPos.setContinuous(false);
        pidPos.setPercentTolerance(Constants.kGrabberPercentTolerance);
    }

    /**
     * Starts moving the arm motor forwards at a given percent output for homing
     */
    public void beginHoming() {
        armMotor.set(Constants.kGrabberHomingSpeed);
        averageCurrent = LinearDigitalFilter.movingAverage(Robot.grabber.getGrabberCurrent(), Constants.kHomeEncoderCurrentCycle);
    }

    /**
     * returns true if the grabber exceeds the current limit from constants
     */
    public boolean exceedsCurrentLimit() {
        return armMotor.getOutputCurrent() > (Constants.kGrabberMaxHomingCurrent + averageCurrent.get()); 
    }

    /**
     * Sets the position offset of the grabber such that newPos is the current position
     */
    public void setEncoderPosition(double newPos) {
        positionOffset = armMotor.getEncoder().getPosition() - newPos;
    }

    /**
     * Gets the current position of the grabber, with corrections from homing (in rotations)
     */
    public double getPosition() {
        return armMotor.getEncoder().getPosition() - positionOffset;
    }

    /**
     * Sets the current state of the grabber
     * @param state the desired state of the grabber
     */
    public void setState(GrabberState state) {
        if (state == GrabberState.EXTENDED) {
            Robot.grabber.getPIDPos().setSetpoint(positionOffset + Constants.kGrabberExtendedPos);
        } else if (state == GrabberState.RETRACTED) {
            Robot.grabber.getPIDPos().setSetpoint(positionOffset + Constants.kGrabberRetractedPos);
        } else if (state == GrabberState.START_POS) {
            Robot.grabber.getPIDPos().setSetpoint(positionOffset + Constants.kGrabberStartPos);
        } else {
            System.out.println("Attepted to set the grabber to an ineligible position");
        }
    }

    /**
     * Get the current state of the grabber
     * @return the current state of the grabber
     */
    public GrabberState getState() {
        if (Math.abs(this.getPosition() - Constants.kGrabberExtendedPos) < Constants.kGrabberTolerance) {
            return GrabberState.EXTENDED;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberRetractedPos) < Constants.kGrabberTolerance) {
            return GrabberState.RETRACTED;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberStartPos) < Constants.kGrabberTolerance) {
            return GrabberState.START_POS;
        } else if (Math.abs(this.getPosition() - Constants.kGrabberHomePos) < Constants.kGrabberTolerance) {
            return GrabberState.HOMED;
        } else {
            return GrabberState.UNKNOWN;
        }
    }


        /**
     * Sets the current state of the grabber
     * @param state the desired state of the grabber
     */
    public void setMotorState(MotorState state) {
        if (state == MotorState.OFF) {
            this.ballMotor.set(0);
        } else if (state == MotorState.INTAKE_BALL) {
            this.ballMotor.set(Constants.kGrabberIntakeSpeed);
        } else if (state == MotorState.EJECT_BALL) {
            this.ballMotor.set(Constants.kGrabberEjectionSpeed);
        } 
    }

    /**
     * Returns if the limit switch for cargo on the grabber is currently pressed 
     */
    public boolean hasCargo() {
        return this.cargoLimitSwitch.get();
    }

    /**
     * Returns if the limit switch for hatches on the grabber is currently pressed 
     */
    public boolean hasHatch() {
        return this.hatchLimitSwitch.get();
    }

    @Override
    protected void initDefaultCommand() {
    }

    public CANSparkMax getGrabberMotor(){
        return armMotor;
    }

    public PIDSource getGrabberCurrent(){
        return this.currentSource;
    }

    public PIDController getPIDPos(){
        return pidPos;
    }

}
