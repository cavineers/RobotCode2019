package frc.lib.tests;

import java.util.ArrayList;

import com.rokus.motorcontroller.Dubins;
import com.rokus.motorcontroller.DubinsTrajectory;
import com.rokus.motorcontroller.geometry.Geometry;

import frc.lib.RobotPos;
import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.RobotCmd;
import frc.lib.pathPursuit.Segment;
import frc.robot.Constants;

public class DubinPathTest {

    static double minRad = 30;

    static Point startPoint = new Point(0, 0);
    static double startHeading = 0;

    static Point endPoint = new Point(20, 3);
    static double endHeading = 0;//Math.PI/8; //fails at 2.03540569779

    public static void main(String[] args) {
        Dubins dubins = new Dubins();

        dubins.setMinTurnRadius(minRad);
        DubinsTrajectory traj = null;
        try {
            traj = dubins.getShortestPath(new Geometry.Pose2d(startPoint.getX(), startPoint.getY(), startHeading), new Geometry.Pose2d(endPoint.getX(), endPoint.getY(), endHeading));
        } catch (Exception e) {
            System.out.println(e);
            return;
        }
        ArrayList<Segment> segList = traj.getSegmentList();
        Path path = new Path(true, false, 1);

        for (Segment seg : segList) {
            System.out.println(seg.getStartPoint() + "," + seg.getEndPoint());
            path.addSegment(seg);
        }

        RobotPos currentPos = new RobotPos(0,0, 0, 0,0);
		
		double pathTime = 0;
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

}