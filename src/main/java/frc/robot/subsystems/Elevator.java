package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Grabber.GrabberPosition;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The Elevator subsystem 
 */
public class Elevator extends Subsystem {
    public CANSparkMax elevatorMotor = new CANSparkMax(RobotMap.elevatorMotor, MotorType.kBrushless);
    DigitalInput limitSwitch;
    private PIDController pidPos;
    double output;
    private boolean isHomed = false;
    
    public enum ElevatorLevel {        
        GROUND,
        LVL1_HATCH,
        CARGOSHIP_TOP,
        LVL1_CARGO,
        LVL2_HATCH,
        LVL2_CARGO,
        LVL3_HATCH,
        LVL3_CARGO,
        INVALID;
    }

    public Elevator() {
        limitSwitch = new DigitalInput(RobotMap.kElevatorLimitSwitch);
        // set PID coefficients
        this.getElevatorMotor().getPIDController().setP(Constants.kPVelocityElev);
        this.getElevatorMotor().getPIDController().setI(Constants.kIVelocityElev);
        this.getElevatorMotor().getPIDController().setD(Constants.kDVelocityElev);
        this.getElevatorMotor().getPIDController().setOutputRange(-1, 1);
        this.getElevatorMotor().setIdleMode(IdleMode.kBrake);
        this.getElevatorMotor().setInverted(false);
        
        pidPos = new PIDController(Constants.kPPosElev, Constants.kIPosElev, Constants.kDPosElev, Constants.kFPosElev, new PIDSource() {
            PIDSourceType vel_sourceType = PIDSourceType.kDisplacement;

            @Override
            public double pidGet() {
                return elevatorMotor.getEncoder().getPosition();
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
                setVelocity(d);
                setPIDPosOutput(d);
            }

        }, Constants.kElevPIDPosPeriod);

        pidPos.setInputRange(Constants.kElevatorMinHeight, Constants.kElevatorMaxHeight);
        pidPos.setOutputRange(-Constants.kElevatorMaxSpeed, Constants.kElevatorMaxSpeed);
        pidPos.setContinuous(false);
        pidPos.setPercentTolerance(Constants.kElevPercentTolerance);
    }

    public double getPIDPosOutput(){
        return output;
    }

    public void setPIDPosOutput(double newOutput){
        output = newOutput;
    }


    @Override
    public void initDefaultCommand() {
    }

    /**
     * get the talon of elevator motor
     * @return talon of elevator motor
     */
    public CANSparkMax getElevatorMotor() {
        return this.elevatorMotor;
    }
    
    /**
     * returns the limit switch responcible for elevator homing
     * @return true if the limit switch is pressed
     */
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }
    
    /**
     * Sets the percent output of the elevator motor
     */
    public void setMotorOutput(double output) {
        this.getElevatorMotor().set(output);
    }

    /**
     * return the outer (position) controller of the cascading pid system
     */
    public PIDController getPIDPos(){
        return pidPos;
    }

    /**
     * Returns whether the grabber can safely move back at the current elevator position
     */
    public boolean canMoveGrabber() {
        return this.getLevel() == ElevatorLevel.GROUND || !this.isHomed() || !Robot.grabber.isHomed();
    }

    /**
     * Moves the elevator to the given setpoint (in rotations)
     */
    public void moveElevator(double p){
        System.out.println("new setpoint: " + p);
        getPIDPos().setSetpoint(p);
    }

    /**
     * Moves the elevator to the given level
     */
    public void moveElevator(ElevatorLevel level) {
        switch (level) {
            case GROUND:
                this.moveElevator(Constants.kElevatorGroundLvl);
                break;
            case CARGOSHIP_TOP:
                this.moveElevator(Constants.kElevatorCargoShipTopRotations);
                break;
            case LVL1_HATCH:
                this.moveElevator(Constants.kElevatorLvl1HatchRotations);
                break;
            case LVL1_CARGO:
                this.moveElevator(Constants.kElevatorLvl1CargoRotations);
                break;
            case LVL2_HATCH:
                this.moveElevator(Constants.kElevatorLvl2HatchRotations);
                break;
            case LVL2_CARGO:
                this.moveElevator(Constants.kElevatorLvl2CargoRotations);
                break;
            case LVL3_HATCH:
                this.moveElevator(Constants.kElevatorLvl3HatchRotations);
                break;
            case LVL3_CARGO:
                this.moveElevator(Constants.kElevatorLvl3CargoRotations);
                break;
            case INVALID:
                System.out.println("cannot raise the elevator to an invalid level");
                break;
        }
    }

    /**
     * Sets the position offset of the elevator such that newPos is the current position
     */
    public void setEncoderPosition(double newPos) {
         elevatorMotor.getEncoder().setPosition(newPos);
    }

    /**
     * Gets the position of the elevator in rotations, with the offset from homing
     */
    public double getPosition() {
        return elevatorMotor.getEncoder().getPosition();
    }

    public void freezeElevator(){
        pidPos.setSetpoint(this.getPosition());
    }

    /**
     * Gets the current level of the elevator, or invalid if the elevator is in between levels
     */
    public ElevatorLevel getLevel() {
        if (Math.abs(this.getPosition() - Constants.kElevatorGroundLvl) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.GROUND;
        } else if (Math.abs(this.getPosition() - Constants.kElevatorLvl1HatchRotations) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.LVL1_HATCH;
        } else if (Math.abs(this.getPosition() - Constants.kElevatorLvl1CargoRotations) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.LVL1_CARGO;    
        } else if (Math.abs(this.getPosition() - Constants.kElevatorLvl2HatchRotations) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.LVL2_HATCH;
        } else if (Math.abs(this.getPosition() - Constants.kElevatorLvl2CargoRotations) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.LVL2_CARGO;
        } else if (Math.abs(this.getPosition() - Constants.kElevatorLvl3HatchRotations) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.LVL3_HATCH;
        } else if (Math.abs(this.getPosition() - Constants.kElevatorLvl3CargoRotations) < Constants.kElevatorPosTolerance) {
            return ElevatorLevel.LVL3_CARGO;
        } else {
            return ElevatorLevel.INVALID;
        }
    }

    public void setVelocity(double vel){
        if(vel>0) {
            this.getElevatorMotor().getPIDController().setFF(Constants.kFVelocityElevUp);
        } else {
            this.getElevatorMotor().getPIDController().setFF(Constants.kFVelocityElevDown);
        }
        this.getElevatorMotor().getPIDController().setReference(vel, ControlType.kVelocity);
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void setHomed(boolean homed) {
        this.isHomed = homed;
    }


}