package frc.lib.tests;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.rokus.motorcontroller.Dubins;
import com.rokus.motorcontroller.DubinsTrajectory;
import com.rokus.motorcontroller.geometry.Geometry;

import frc.lib.RobotPos;
import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Lookahead;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.RobotCmd;
import frc.lib.pathPursuit.Segment;
import frc.robot.Constants;

public class DubinPathTest {

    static double minRad = 3;

    static Point startPoint = new Point(0, 0);
    static double startHeading = 0;

    static Point endPoint = new Point(10, -7);
    static double endHeading = 2.03540569779;

    public static void main(String[] args) {
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        // NetworkTable table = inst.getTable("datatable");
        // NetworkTableEntry xEntry = table.getEntry("x");
        // NetworkTableEntry yEntry = table.getEntry("y");
        // inst.startClientTeam(4541);  // where TEAM=190, 294, etc, or use inst.startClient("hostname") or similar
        // inst.startDSClient(); 


        RobotPos currentPos = new RobotPos(0,0, 0, 0,0);
        
        double pathTime = 0;
        
        Path path = getPath(startPoint, startHeading, endPoint, endHeading);

        while (!path.isFinished()) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            RobotCmd cmd = path.update(currentPos);
            double heading = currentPos.heading + ((cmd.getLeftVel() - cmd.getRightVel()) / Constants.kWheelBase) * path.manager.dt;
            
            double lVel = cmd.getLeftVel();
            double rVel = cmd.getRightVel();
            
            double xPos = currentPos.position.getX() + (lVel + rVel)/2 * path.manager.dt * Math.cos(heading);
            double yPos = currentPos.position.getY() + (lVel  + rVel)/2 * path.manager.dt * Math.sin(heading);
            currentPos = new RobotPos(xPos, yPos, heading, rVel, lVel);
            pathTime += Constants.kDefaultDt;
            // System.out.println(currentPos);
            // System.out.println(path.getCurrentSegment()); 
        }

        System.out.println("Estimate time Required " + pathTime + " sec");

        



        // System.out.println("Executing getShortestPath");
        // DubinsTrajectory DT = dubins.getShortestPath(new Geometry.Pose2d(spx,spy,spt), new Geometry.Pose2d(gpx,gpy,gpt));
        // System.out.println("Trajectory:\n Type = " + DT.type + "\n Controls = \nSteering radius \t\tTime steps");

    }

    public static Path getPath(Point start, double startHeading, Point end, double endHeading) {
        Dubins dubins = new Dubins();
        dubins.setMinTurnRadius(minRad);
        DubinsTrajectory traj = null;

        double straightLineAmount = 3;
        double x = end.getX() - straightLineAmount * Math.cos(endHeading);
        double y = end.getY() - straightLineAmount * Math.sin(endHeading);
        Point dubinEnd = new Point(x,y);

        try {
            traj = dubins.getShortestPath(new Geometry.Pose2d(startPoint.getX(), startPoint.getY(), startHeading), new Geometry.Pose2d(dubinEnd.getX(), dubinEnd.getY(), endHeading));
        } catch (Exception e) {
            System.out.println(e);
            return null;
        }
        ArrayList<Segment> segList = traj.getSegmentList();
        Path path = new Path(true, false, Constants.kMaxTargetAccel,  new Lookahead(Constants.kMinLookAheadTargeting, Constants.kMinLookAheadTargeting, Constants.kMinLookAheadSpeedTargeting, Constants.kMaxLookAheadSpeedTargeting));
        for (Segment seg : segList) {
            path.addSegment(seg);
        }
        path.addSegment(new LineSegment(dubinEnd, end, Constants.kMaxTargetSpeed, 0));
        return path;
    }

}