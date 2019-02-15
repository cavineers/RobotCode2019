/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.pathPursuit.Path;
import frc.robot.commands.FollowPath;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.TargetVisionTape;
import frc.robot.commands.FollowPath.PATH_TYPE;
import frc.robot.subsystems.DriveTrain.DriveGear;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber.LegState;
import frc.robot.commands.ChangeClimberState;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Rumble;
import frc.robot.commands.Rumble.ControllerSide;
import frc.robot.commands.elevator.HomeElev;
import frc.robot.commands.elevator.ManualElev;
import frc.robot.commands.elevator.ElevatorToLevel;
import frc.robot.LEDHelper;
import frc.robot.LEDHelper.LEDColor;





/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public static Joystick joy = new Joystick(0);
    public static JoystickButton a_button = new JoystickButton(joy, 1);
    public static JoystickButton b_button = new JoystickButton(joy, 2);
    public static JoystickButton x_button = new JoystickButton(joy, 3);
    public static JoystickButton y_button = new JoystickButton(joy, 4);

    public static JoystickButton l_bump = new JoystickButton(joy, 5);
    public static JoystickButton r_bump = new JoystickButton(joy, 6);
    public static JoystickButton left_middle = new JoystickButton(joy, 7);
    public static JoystickButton right_middle = new JoystickButton(joy, 8);
    public static JoystickButton left_stick = new JoystickButton(joy, 9);
    public static JoystickButton right_stick = new JoystickButton(joy, 10);

    LEDHelper led;

    public int lastDpad = -1;

    public enum TRIG_MODE {
		ELEVATOR, CLIMBER
    }

    public TRIG_MODE currentTriggerSetting = TRIG_MODE.ELEVATOR;
    
    public OI() {
        r_bump.whenPressed(new ShiftGear(DriveGear.HIGH_GEAR)); // right is high
        l_bump.whenPressed(new ShiftGear(DriveGear.LOW_GEAR)); // left is low
        // a_button.whenPressed(new TargetVisionTape());
        // b_button.whenPressed(new ElevatorToPos(10));
        // x_button.whenPressed(new ChangeClimberState(LegState.DEPLOYED));
        // y_button.whenPressed(new ToggleCargoIntake());

        
        //testing only
        /*x_button.whenPressed(new HomeElev());
        y_button.whenPressed(new ElevatorToLevel(ElevatorLevel.LVL1_CARGO)); //3 inches
        a_button.whenPressed(new Command() {
            protected void initialize() { 
               Robot.elevator.getElevatorMotor().set(.3);
             }
            @Override
            protected boolean isFinished() {
                return true;
            }
            
        });

        b_button.whenPressed(new Command() {
            protected void initialize() { 
               Robot.elevator.getElevatorMotor().stopMotor();
             }
            @Override
            protected boolean isFinished() {
                return true;
            }
            
        });*/

        a_button.whenPressed(getAButton());
        y_button.whenPressed(getYButton());
       
        System.out.println("Motor Rotations: " + Robot.elevator.getElevatorMotor().getEncoder().getPosition());

        left_middle.whenPressed(new Command() { //Toggle between elevator and climber
            protected void initialize() { 
               if (Robot.oi.currentTriggerSetting == TRIG_MODE.ELEVATOR && Robot.isEndGame()) {
                   Robot.oi.currentTriggerSetting = TRIG_MODE.CLIMBER;
                   new Rumble(0.25, ControllerSide.BOTH).start();
                   led.setLEDColor(LEDColor.PURPLE);
                   Robot.climber.toggleArms();
               } else {
                   Robot.oi.currentTriggerSetting = TRIG_MODE.ELEVATOR;
               }
            }
           @Override
           protected boolean isFinished() {
               return true;
           }
           
       });

    }

    

    public void getDPad(){
        if (lastDpad != joy.getPOV()) {
			switch (joy.getPOV()) {
			case 0: {
                // Top
                if(Robot.grabber.hasCargo()){
                    Robot.elevator.moveElevator(ElevatorLevel.LVL3_CARGO);
                }
                else if(Robot.grabber.hasHatch()){
                    Robot.elevator.moveElevator(ElevatorLevel.LVL3_HATCH);
                }
                else{
                    Robot.elevator.moveElevator(ElevatorLevel.LVL3_CARGO); 
                }
				break;
			}
			case 90: {
				// Right
                if(Robot.grabber.hasCargo()){
                    Robot.elevator.moveElevator(ElevatorLevel.LVL2_CARGO);
                }
                else if(Robot.grabber.hasHatch()){
                    Robot.elevator.moveElevator(ElevatorLevel.LVL2_HATCH);
                }
                else{
                    Robot.elevator.moveElevator(ElevatorLevel.LVL2_CARGO); 
                }
				break;
			}
			case 180: {
				// Bottom
				Robot.elevator.moveElevator(ElevatorLevel.GROUND);
				break;
			}
			case 270: {
				// Left
				if(Robot.grabber.hasCargo()){
                    Robot.elevator.moveElevator(ElevatorLevel.LVL1_CARGO);
                }
                else if(Robot.grabber.hasHatch()){
                    Robot.elevator.moveElevator(ElevatorLevel.LVL1_HATCH);
                }
                else{
                    Robot.elevator.moveElevator(ElevatorLevel.LVL1_CARGO); 
                }
				break;
			}
			}
		}
		lastDpad = joy.getPOV();
    }
       

    public Joystick getJoystick() {
        return joy;
    }

    public double addDeadZone(double input) {
        if (Math.abs(input) <= .05)
            input = 0;
        else if (input < 0)
            input = -Math.pow(input, 2);
        else
            input = Math.pow(input, 2);
        return input;
    }

    


    public Command getAButton(){
        if(currentTriggerSetting == TRIG_MODE.CLIMBER){
            return new ChangeClimberState(LegState.DEPLOYED);
        }
        else{
            return new IntakeCargo();
        }
    }

    public Command getYButton(){
        if(currentTriggerSetting == TRIG_MODE.CLIMBER){
            return new ChangeClimberState(LegState.RETRACTED);
        }
        else{
            return new Command() {
                protected void initialize() { 
                    isFinished();
                 }
                @Override
                protected boolean isFinished() {
                    return true;
                }
                
            };
        }
    }

    
}
