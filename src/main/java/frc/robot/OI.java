/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.EnterDefenseMode;
import frc.robot.commands.HomeAll;
import frc.robot.commands.IntakeCargo;
import frc.robot.commands.MoveGrabberAndElevator;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.grabber.EjectBall;
import frc.robot.commands.grabber.ToggleGrabber;
import frc.robot.commands.grabber.ToggleHatchGrabber;
import frc.robot.commands.vision.VCargo;
import frc.robot.commands.vision.VCargo.VCargoHeight;
import frc.robot.subsystems.DriveTrain.DriveGear;
import frc.robot.subsystems.Elevator.ElevatorLevel;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Grabber.GrabberPosition;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    LEDHelper led;

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

    public int lastDpad = -1;
    public boolean lastRightTrig = false;
    public boolean lastLeftTrig = false;

    public enum BUTTON_MODE {
		AUTO_ALLIGN, ELEVATOR, CLIMBER
    }

    public BUTTON_MODE currentTriggerSetting = BUTTON_MODE.AUTO_ALLIGN;
    
    public OI() {
        
        r_bump.whenPressed(new ShiftGear(DriveGear.LOW_GEAR));
        l_bump.whenPressed(new ShiftGear(DriveGear.HIGH_GEAR));

        //actual button commands
        a_button.whenPressed(new IntakeCargo());
        b_button.whenPressed(new ToggleGrabber());
        x_button.whenPressed(new ToggleHatchGrabber());
        y_button.whenPressed(new EjectBall());

        left_middle.whenPressed(new HomeAll());
        // right_middle.whenPressed(new ToggleCargoIntake());
        right_middle.whenPressed(new VCargo());

        // Homing test
        // a_button.whenPressed(new HomeElev());

        // Raw % output elevator controls
        // a_button.whileHeld(new RawMoveElevator(0.25));
        // y_button.whileHeld(new RawMoveElevator(-0.25));

        //Raw velocity control
        // x_button.whileHeld(new RawSetElevatorVelocity(1500));
        // b_button.whileHeld(new RawSetElevatorVelocity(-1500));

        //Raw Cascading PID control
        // x_button.whileHeld(new RawSetElevatorPosition(75));
        // y_button.whileHeld(new RawSetElevatorPosition(150));


        //Grabber velocity Control
        // x_button.whileHeld(new RawSetGrabberVelocity(1000));
        // y_button.whileHeld(new RawSetGrabberVelocity(-1000));
        // a_button.whenPressed(new HomeGrabber());

        // x_button.whileHeld(new RawSetGrabberPosition(0));
        // y_button.whileHeld(new RawSetGrabberPosition(10));


    }

    public boolean isRightTriggerPressed() {
        double rightTrig = this.getJoystick().getRawAxis(3);
        return rightTrig > 0.9;
    }

    public boolean isLeftTriggerPressed() {
        double leftTrig = this.getJoystick().getRawAxis(2);
        return leftTrig > 0.9;
    }

    public void updatePeriodicCommands(){
        if (lastRightTrig != isRightTriggerPressed()) {
            // the right trigger changed state
            lastRightTrig = isRightTriggerPressed();
            if (lastRightTrig) {
                // the right trigger is pressed
                new EnterDefenseMode().start();
            }
        }

        if (lastLeftTrig != isLeftTriggerPressed()) {
            // the left trigger changed state
            lastLeftTrig = isLeftTriggerPressed();
            if (lastLeftTrig) {
                // the left trigger is pressed
                // new MoveGrabberAndElevator(ElevatorLevel.CARGOSHIP_TOP, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                new VCargo();

            }
        }

        if (lastDpad != joy.getPOV()) {
			switch (joy.getPOV()) {
			case 0: {
                // Top
                if(Robot.grabber.hasCargo()){
                    new MoveGrabberAndElevator(ElevatorLevel.LVL3_CARGO, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
                else if(Robot.grabber.hasHatch()){
                    new MoveGrabberAndElevator(ElevatorLevel.LVL3_HATCH, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
                else{
                    new MoveGrabberAndElevator(ElevatorLevel.LVL3_HATCH, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
				break;
			}
			case 90: {
				// Right
                if(Robot.grabber.hasCargo()){
                    new MoveGrabberAndElevator(ElevatorLevel.LVL2_CARGO, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();

                }
                else if(Robot.grabber.hasHatch()){
                    new MoveGrabberAndElevator(ElevatorLevel.LVL2_HATCH, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
                else{
                    new MoveGrabberAndElevator(ElevatorLevel.LVL2_HATCH, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
				break;
			}
			case 180: {
                // Bottom
                new MoveGrabberAndElevator(ElevatorLevel.GROUND, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                break;
            }
			case 270: {
				// Left
				if(Robot.grabber.hasCargo()){
                    new MoveGrabberAndElevator(ElevatorLevel.LVL1_CARGO, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
                else if(Robot.grabber.hasHatch()){
                    new MoveGrabberAndElevator(ElevatorLevel.LVL1_HATCH, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
                }
                else{
                    new MoveGrabberAndElevator(ElevatorLevel.LVL1_HATCH, GrabberPosition.EXTENDED, Grabber.MotorState.OFF).start();
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
