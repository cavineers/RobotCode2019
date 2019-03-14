/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  // CAN IDs

  //drive motors
  public static int rightDriveMotor1 = 0; //talon (cim) (40A)
  public static int rightDriveMotor2 = 1; //talon (cim) (40A)
  public static int leftDriveMotor1  = 2; //talon (cim) (40A)
  public static int leftDriveMotor2  = 3; //talon (cim) (40A)

  //elevator motors
  public static int elevatorMotor = 4; //sparkmax (neo) (40A) //TODO 4

  //cargo intake motor
  public static int intakeMoter = 5; //talon (775 pro) (30A)
  
  //climber winch motor
  public static int climberMotor = 6; //sparkmax (neo) (40A)

  // arm movement motor
  public static int armMotor = 7; // sparkmax (neo) (40A)

  // grabber intake motor
  public static int grabberIntake = 8; //talon (bag) (30A)

  //pneumatics:

  //PCM can IDs
  public static int PCM1 = 9;

  public static int PDP = 10;

  //PCM devices:

  //Drive Shifters (switches between high and low drive gears)
  public static int driveShifter1 = 4;
  public static int driveShifter2 = 3;

  //Hatch Scoop (lowers and raises the hatch scooping arms)
  public static int hatchScoop1 = 5;
  public static int hatchScoop2 = 2;

  //Grabber (controls both hatch intake and ball hard stop)
  public static int grabber1 = 6;
  public static int grabber2 = 1;

  //Cargo Intake (controls if the cargo intake arms are down or not)
  public static int cargoIntake1 = 7;
  public static int cargoIntake2 = 0;

  //Digital In/Output mapping

  //auto selection
  public static final int kAutoPos0 = 0;
  public static final int kAutoPos1 = 1;
  public static final int kAutoPos2 = 2;
  public static final int kAutoPos3 = 3;
  public static final int kAutoPos4 = 4;
  public static final int kDIO5 = 5;
  public static final int kGrabberHomeingLimitSwitch = 6;
  public static final int kElevatorLimitSwitch = 7;
  public static final int kGrabberHatchLimitSwitch = 8;
  public static final int kGrabberCargoLimitSwitch = 9;
  
}
