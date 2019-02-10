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

  //drive motors
  public static int rightDriveMotor1 = 0;
  public static int rightDriveMotor2 = 1;
  public static int leftDriveMotor1  = 2;
  public static int leftDriveMotor2  = 3;

  //elevator motors
  public static int elevatorMotor1 = 4;
  public static int elevatorMotor2 = 5;

  public static int intakeMoter = 6;

  public static int climberMotor = 7;

  //pneumatics
  public static int PCM1 = 8;
  public static int PCM2 = 9;
}
