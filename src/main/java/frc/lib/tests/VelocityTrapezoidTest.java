package frc.lib.tests;

import frc.lib.VelocityTrapezoid;
import frc.robot.Constants;

public class VelocityTrapezoidTest {
    static double currentPos = 0;
    static double currentVel = 0;
    static double setpoint = 100;
    static int loops = 1000;
    public static void main (String[] args) {
        VelocityTrapezoid velTrapezoid = new VelocityTrapezoid(Constants.kElevatorMaxAcceleration, Constants.kElevatorMaxSpeed, Constants.kDefaultDt);
        int i = 0;
        velTrapezoid.setDebugMode();
        velTrapezoid.setDecelLock(false);
        while (i < loops) {
            i++;
            double error = setpoint - currentPos;
            currentVel = velTrapezoid.update(currentVel, error);
            currentPos += currentVel * Constants.kDefaultDt;
            System.out.println(error + "," + currentVel);
        }

    }
}