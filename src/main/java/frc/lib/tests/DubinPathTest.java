package frc.lib.tests;

import java.util.ArrayList;
import java.util.Random;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.dubinPath.DubinPathCalculator;
import frc.lib.dubinPath.Dubins;
import frc.lib.dubinPath.DubinsPath;
import frc.lib.dubinPath.DubinsTrajectory;
import frc.lib.dubinPath.TrajectoryType;
import frc.lib.dubinPath.geometry.Geometry;

import frc.lib.MathHelper;
import frc.lib.RobotPos;
import frc.lib.RobotPosMap;
import frc.lib.RobotPosUpdate;
import frc.lib.TargetUpdate;
import frc.lib.Vector2D;
import frc.lib.Vector3D;
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

    static Point startPoint = new Point(-0.07724603391119139,0.0012551598892283694);
    static double startHeading = -0.024783674629305422;

    // static Point endPoint = new Point(37.3861478587509, 8.268122838004851);
    // static double endHeading = 0.2127759890603868;//2.3573910734479835;
    static double straightLineAmount = 3;

    static RobotPosMap map = new RobotPosMap(200, new RobotPosUpdate(startPoint.getX(), startPoint.getY(), startHeading, 0, UpdateType.BASE));

    public static void main(String[] args) {
        double pathTime = 0;

        double startVel = 0;
        
        Path path = createPath(new TargetUpdate(0.0155049936645,0.280920635208,0.959605748152,0.63722241386,-4.11429654723,23.8722069873,-0.425705147813,2.78770529811,24.0678879262,204,156.7497000694275));

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
            
            // if (r.nextInt(5) == 0) {
            //     map.addCameraUpdate(currentPos.getX() + MathHelper.getRandomDouble(-1.5, 1.5), currentPos.getY() + MathHelper.getRandomDouble(-1, 1), currentPos.heading, pathTime);
            // }
            pathTime += Constants.kDefaultDt;

            System.out.println(currentPos);

        }

        System.out.println("Estimate time Required " + pathTime + " sec");
    }

    protected static Path createPath(TargetUpdate targetUpdate) {
        System.out.println("Found Target");

        RobotPos robotFieldPos = new RobotPos(-0.07724603391119139,0.0012551598892283694,  -0.024783674629305422,  0,0); // the position of the robot when the picture was taken (field coords)
        
        System.out.println("Pos at time: " + robotFieldPos);

        //convert the target heading vector to the robot's coordinate system
        Vector3D targetHeadingVect = targetUpdate.getHeadingVector();
        targetHeadingVect.rotate(Constants.kCameraToRobotMatrix);

        System.out.println("Target Heading Vect: " + targetHeadingVect);

        // compute the target heading
        double targetHeading = robotFieldPos.getHeading() + Math.atan(targetHeadingVect.getDx() / targetHeadingVect.getDy());

        System.out.println("Target Heading: " + targetHeading);

        // the target in the robot's reference frame
        Vector3D targetRobotFrameRobotOrigin = Vector3D.add(Constants.kCameraRelativeToRobotCenter, targetUpdate.getCameraVector().rotate(Constants.kCameraToRobotMatrix)); 

        //the target in a coordinate system alligned with the field, but centered at the robot
        Vector3D targetFieldFrameRobotOrigin = targetRobotFrameRobotOrigin.rotateZAxis(-robotFieldPos.getHeading());

        System.out.println("Robot Oriented Robot Origin: " + targetRobotFrameRobotOrigin);

        System.out.println("Field Oriented Robot Origin: " + targetFieldFrameRobotOrigin);

        // rotate the vector such that its x component is pointing forward to match path pursuit's coordinate system
        // targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(Math.PI/2);
        // targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateYAxis(Math.PI);

        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateZAxis(Math.PI/2);
        targetFieldFrameRobotOrigin = targetFieldFrameRobotOrigin.rotateXAxis(Math.PI);

        //the target's location relative to the field in 2 dimentions
        Vector2D targetFieldLocation = new Vector2D(targetFieldFrameRobotOrigin.getDx() + robotFieldPos.getX(), targetFieldFrameRobotOrigin.getDy() + robotFieldPos.getY());

        System.out.println("Target Field Location: " + targetFieldLocation.toString());

        RobotPos latestFieldPos = new RobotPos(0,0,  0,  0,0);

        DubinsPath dubinsPath = DubinPathCalculator.getBestPath(latestFieldPos.position, latestFieldPos.getHeading(), targetFieldLocation.getPoint(), targetHeading);
        
        if (!dubinsPath.isValid()) {
            System.out.println("INVALID PATH");
        }
        System.out.println("planned path");
        return dubinsPath.getPath();
    }
}