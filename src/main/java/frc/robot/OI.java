/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.lib.pathPursuit.Path;
import frc.robot.commands.FollowPath;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.TargetVisionTape;
import frc.robot.commands.FollowPath.PATH_TYPE;
import frc.robot.subsystems.DriveTrain.DriveGear;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber.LegState;
import frc.robot.commands.ChangeClimberState;
import frc.robot.commands.ElevatorToPos;
import frc.robot.commands.ToggleGrabber;
import com.ctre.phoenix.motorcontrol.ControlMode;

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
    public int lastDpad = -1;

    public OI() {
        r_bump.whenPressed(new ShiftGear(DriveGear.HIGH_GEAR)); // right is high
        l_bump.whenPressed(new ShiftGear(DriveGear.LOW_GEAR)); // left is low
        a_button.whenPressed(new TargetVisionTape());
        b_button.whenPressed(new ElevatorToPos(10));
        x_button.whenPressed(new ChangeClimberState(LegState.DEPLOYED));
        y_button.whenPressed(new ToggleGrabber());
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
