package frc.lib.tests;

import java.util.ArrayList;
import java.util.Random;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.dubinPath.DubinPathCalculator;
import frc.lib.dubinPath.Dubins;
import frc.lib.dubinPath.DubinsTrajectory;
import frc.lib.dubinPath.TrajectoryType;
import frc.lib.dubinPath.geometry.Geometry;

import frc.lib.MathHelper;
import frc.lib.RobotPos;
import frc.lib.RobotPosMap;
import frc.lib.RobotPosUpdate;
import frc.lib.RobotPosUpdate.UpdateType;
import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Lookahead;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.RobotCmd;
import frc.lib.pathPursuit.Segment;
import frc.robot.Constants;

public class DubinPathTest {

    // static double minRad = 30;

    static Point startPoint = new Point(0, 0);
    static double startHeading = Math.PI;

    static Point endPoint = new Point(-22.7227053308, 9.873755356459998);
    static double endHeading = -7.400670117941084E-4 + Math.PI;
    static double straightLineAmount = 1;

    static RobotPosMap map = new RobotPosMap(200, new RobotPosUpdate(startPoint.getX(), startPoint.getY(), startHeading, 0, UpdateType.BASE));

    public static void main(String[] args) {
        double pathTime = 0;

        double startVel = 40;
        
        Path path = DubinPathCalculator.getBestPath(startPoint, startHeading, endPoint, endHeading).getPath();

        RobotPos currentPos = map.getLastestFieldRelativePosition();
        currentPos.setVelocities(startVel, startVel);

        // Random r = new Random();

        double maxSpeedForTesting = 30000;

        while (!path.isFinished()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            RobotCmd cmd = path.update(currentPos);

            double lVel = cmd.getLeftVel();
            if (lVel > maxSpeedForTesting) {
                lVel = maxSpeedForTesting;
            } else if (lVel < -maxSpeedForTesting) {
                lVel = -maxSpeedForTesting;
            }
            double rVel = cmd.getRightVel();
            if (rVel > maxSpeedForTesting) {
                rVel = maxSpeedForTesting;
            } else if (rVel < -maxSpeedForTesting) {
                rVel = -maxSpeedForTesting;
            }

            double heading = currentPos.heading + ((cmd.getLeftVel() - cmd.getRightVel()) / Constants.kWheelBase) * path.manager.dt;
            
            double dx = (lVel + rVel)/2 * path.manager.dt * Math.cos(heading);
            double dy = (lVel  + rVel)/2 * path.manager.dt * Math.sin(heading);

            // currentPos = new RobotPos(xPos, yPos, heading, rVel, lVel);
            map.addWheelUpdate(dx, dy, heading, pathTime);

            currentPos = map.getLastestFieldRelativePosition();
            currentPos.setVelocities(rVel, lVel);
            
            //if the robot is at the end of the path, stop incorperating camera updates
            if (!path.canMoveOnToNextSegment()) {
                map.setAllowedToIntegrateCameraUpdates(false);
            }

            // if (r.nextInt(5) == 0) {
            //     map.addCameraUpdate(currentPos.getX() + MathHelper.getRandomDouble(-1.5, 1.5), currentPos.getY() + MathHelper.getRandomDouble(-1, 1), currentPos.heading, pathTime);
            // }
            pathTime += Constants.kDefaultDt;

            System.out.println(currentPos);

        }

        System.out.println("Estimate time Required " + pathTime + " sec");
    }
}