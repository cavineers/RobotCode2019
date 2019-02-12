package frc.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import frc.robot.subsystems.CargoIntake.positionState;
import frc.robot.subsystems.HatchScoop.HatchScoopState;

public class LEDHelper {
    DigitalOutput redD5, purpleD6, blueD7, orangeD8, greenD9;
    LEDColor currentColor = LEDColor.NONE;
    public LEDHelper () {
        redD5    = new DigitalOutput(5);
        purpleD6 = new DigitalOutput(6);
        blueD7   = new DigitalOutput(7);
        orangeD8 = new DigitalOutput(8);
        greenD9  = new DigitalOutput(9);
        
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
     * Converts a color into the required digital outputs to communcate 
     * the desired color to the arduino
     */
    public void setLEDColor(LEDColor color) {
        switch (color) {
            case BLINKING_RED:
                redD5.set(true);
                purpleD6.set(false);
                blueD7.set(false);
                orangeD8.set(false);
                greenD9.set(false);
                currentColor = LEDColor.BLINKING_RED;
                break;
            case PURPLE:
                redD5.set(false);
                purpleD6.set(true);
                blueD7.set(false);
                orangeD8.set(false);
                greenD9.set(false);
                currentColor = LEDColor.PURPLE;
                break;
            case BLUE:
                redD5.set(false);
                purpleD6.set(false);
                blueD7.set(true);
                orangeD8.set(false);
                greenD9.set(false);
                currentColor = LEDColor.BLUE;
                break;
            case ORANGE:
                redD5.set(false);
                purpleD6.set(false);
                blueD7.set(false);
                orangeD8.set(true);
                greenD9.set(false);
                currentColor = LEDColor.ORANGE;
                break;
            case GREEN:
                redD5.set(false);
                purpleD6.set(false);
                blueD7.set(false);
                orangeD8.set(false);
                greenD9.set(true);
                currentColor = LEDColor.GREEN;
                break;
            case NONE:
                redD5.set(false);
                purpleD6.set(false);
                blueD7.set(false);
                orangeD8.set(false);
                greenD9.set(false);
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
        } else if (Robot.isClimbing()) {
            return LEDColor.PURPLE;
        } else if (Robot.hatchScoop.getState() == HatchScoopState.DOWN) {
            return LEDColor.BLUE;
        } else if (Robot.cargoIntake.getPosition() == positionState.DOWN) {
            return LEDColor.ORANGE;
        } else if (Robot.cameraManager.hasValidCameraUpdate()) {
            return LEDColor.GREEN;
        } else {
            return LEDColor.NONE;
        }
    }
}