package frc.lib.tests;

import frc.lib.RobotPos;
import frc.lib.RobotPosMap;
import frc.lib.RobotPosUpdate;
import frc.lib.VelocityTrapezoid;
import frc.lib.RobotPosUpdate.UpdateType;
import frc.lib.pathPursuit.ConnectionArc;
import frc.lib.pathPursuit.Point;
import frc.robot.Constants;

public class DriveToTargetTestConnectionArc {

    static RobotPosMap map = new RobotPosMap(100, new RobotPosUpdate(0,0,0, 0, UpdateType.BASE));
    static VelocityTrapezoid velTrap = new VelocityTrapezoid(Constants.kMaxAccelSpeedUp, Constants.kMaxTargetSpeed, Constants.kDefaultDt);
    static Point target = new Point(25,25);

    static double loopCount = 0;
    static double currentHeading = 0;

    static double rVel = 0;
    static double lVel = 0;
    public static void main(String[] args) {
        velTrap.setDebugMode();
        while(true) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            double currentVel = (rVel + lVel)/2;

            RobotPos currentPos = map.getLastestFieldRelativePosition();

            ConnectionArc arc = new ConnectionArc(new RobotPos(currentPos.position, currentHeading, 0, 0), target, false);

            double newVel = velTrap.update(currentVel, arc.getLength());
            velTrap.setDecelLock(false);

            lVel = arc.getLeftVelocityTarget(newVel);
            rVel = arc.getRightVelocityTarget(newVel);

            loopCount ++;

            currentHeading += ((lVel - rVel) / Constants.kWheelBase) * Constants.kDefaultDt;
            
            map.addWheelUpdate((lVel + rVel)/2 * Constants.kDefaultDt * Math.cos(currentHeading), (lVel  + rVel)/2 * Constants.kDefaultDt * Math.sin(currentHeading), currentHeading, loopCount * Constants.kDefaultDt);
            
            System.out.printf(String.format("%f,%f,%f,%f,%f\n", currentPos.getX(), currentPos.getY(), currentHeading, lVel, rVel));
        }
    }

}