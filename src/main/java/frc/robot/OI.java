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
import frc.robot.commands.elevator.HomeElev;
import frc.robot.commands.elevator.RawMoveElevator;
import frc.robot.commands.grabber.RawMoveGrabber;
import frc.robot.LEDHelper;

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

    public enum BUTTON_MODE {
		AUTO_ALLIGN, ELEVATOR, CLIMBER
    }

    public BUTTON_MODE currentTriggerSetting = BUTTON_MODE.AUTO_ALLIGN;
    
    public OI() {
        
        r_bump.whenPressed(new ShiftGear(DriveGear.HIGH_GEAR)); // right is high
        l_bump.whenPressed(new ShiftGear(DriveGear.LOW_GEAR)); // left is low
        // a_button.whenPressed(new TargetVisionTape());
        // b_button.whenPressed(new ElevatorToPos(10));
        // x_button.whenPressed(new ChangeClimberState(LegState.DEPLOYED));
        // y_button.whenPressed(new ToggleCargoIntake());

        // a_button.whenPressed(new IntakeCargo());
        // b_button.whenPressed(new ToggleGrabber());
        // x_button.whenPressed(new ToggleHatchGrabber());
        // y_button.whenPressed(new EjectBall());

        a_button.whileHeld(new RawMoveGrabber(0.4));
        y_button.whileHeld(new RawMoveGrabber(-0.4));
        x_button.whenPressed(new HomeElev());
    }

    public void updateDPadCommands(){
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

}
