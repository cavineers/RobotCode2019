package frc.robot;

import javax.management.timer.Timer;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.robot.subsystems.CargoIntake.PositionState;
import frc.robot.subsystems.Grabber.HatchGrabberState;
import frc.robot.subsystems.HatchScoop.HatchScoopState;

public class LEDHelper {
    Relay purpleR0, blueR1, orangeR2, greenR3;
    LEDColor currentColor = LEDColor.NONE;
    public LEDHelper () {
        purpleR0 = new Relay(0);
        blueR1   = new Relay(1);
        orangeR2 = new Relay(2);
        greenR3  = new Relay(3);
        
        this.setLEDColor(LEDColor.NONE);
    }

    enum LEDColor {
        BLINKING_RED, // there is some sort of error with the robot
        PURPLE, // robot is in climber mode
        BLUE,   // robot has the hatch scoop down
        ORANGE, // robot has the intake down
        GREEN,  // robot has a valid target update
        NONE    // none of the above is true; LEDs should not be on
    }

    /**
     * Makes sure that the desired color is the same as the current color
     */
    public void update() {
        
        LEDColor desiredColor = this.getDesiredColor();
        if (desiredColor != this.currentColor) {
            this.setLEDColor(desiredColor);
        }
    }

    /**
     * Converts a color into the required relay outputs to communcate 
     * the desired color to the arduino
     */
    public void setLEDColor(LEDColor color) {
        switch (color) {
            case BLINKING_RED:
                purpleR0.set(Value.kForward);
                blueR1.set(Value.kForward);
                orangeR2.set(Value.kForward);
                greenR3.set(Value.kForward);
                currentColor = LEDColor.BLINKING_RED;
                break;
            case PURPLE:
                purpleR0.set(Value.kForward);
                blueR1.set(Value.kOff);
                orangeR2.set(Value.kOff);
                greenR3.set(Value.kOff);
                currentColor = LEDColor.PURPLE;
                break;
            case BLUE:
                purpleR0.set(Value.kOff);
                blueR1.set(Value.kForward);
                orangeR2.set(Value.kOff);
                greenR3.set(Value.kOff);
                currentColor = LEDColor.BLUE;
                break;
            case ORANGE:
                purpleR0.set(Value.kOff);
                blueR1.set(Value.kOff);
                orangeR2.set(Value.kForward);
                greenR3.set(Value.kOff);
                currentColor = LEDColor.ORANGE;
                break;
            case GREEN:
                purpleR0.set(Value.kOff);
                blueR1.set(Value.kOff);
                orangeR2.set(Value.kOff);
                greenR3.set(Value.kForward);
                currentColor = LEDColor.GREEN;
                break;
            case NONE:
                purpleR0.set(Value.kOff);
                blueR1.set(Value.kOff);
                orangeR2.set(Value.kOff);
                greenR3.set(Value.kOff);
                currentColor = LEDColor.NONE;
                break;
        }
    }

    /**
     * Gets the desired LED color based on the robot's state
     */
    public LEDColor getDesiredColor() {
        if (!Robot.reflectiveTapeCamera.isPiConnected()) { //add other errors
            return LEDColor.BLINKING_RED;
        } else if (Robot.grabber.getHatchGrabberState() == HatchGrabberState.INTAKING) {
            return LEDColor.PURPLE;
        } else if (Robot.hatchScoop.getState() == HatchScoopState.DOWN) {
            return LEDColor.BLUE;
        } else if (Robot.cargoIntake.getPosition() == PositionState.DOWN) {
            return LEDColor.ORANGE;
        } else if (Robot.reflectiveTapeCamera.getUpdate() != null) {
            return LEDColor.GREEN;
        } else {
            return LEDColor.NONE;
        }
    }
}