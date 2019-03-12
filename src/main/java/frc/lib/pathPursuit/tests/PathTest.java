package frc.lib.pathPursuit.tests;

import frc.lib.RobotPos;
import frc.lib.pathPursuit.*;
import frc.robot.AutoPathHelper;
import frc.robot.Constants;

public class PathTest {
    
    /*
     * A class which helped test getClosestPointOnSegment with the help of Desmos Graphing Calculator
     * https://www.desmos.com/calculator/qdsqwrsrke
     */
    public static void main(String[] args) {
//		Segment testSegment = new LineSegment(new Point(-2.5, 6.7), new Point(-4.2, 10), 0);
//		Segment testSegment = new ArcSegment(new Point(2, 0), new Point(0, -2), new Point(0, 0), 0);
//		
//		Point testPoint = new Point(1, -1);
//		Point lookahead = testSegment.getClosestPointOnSegment(testPoint);
//		
//		System.out.println("closest point:" + lookahead);
//		System.out.println("dist 1:" + testSegment.getDistanceToEndpoint(testSegment.getStartPoint()));
//		System.out.println("dist 2:" + testSegment.getDistanceToEndpoint(lookahead));
//		
//		VelocityManager manager = new VelocityManager(Constants.kMaxAccel, Constants.kMaxJerk);
//		double distNeeded = manager.getDistanceNeededToAccel(Constants.kMaxAccel, 30, 0);
//		System.out.println(distNeeded);
        
        // Path path = new Path(true, false, Constants.kMaxAccelSpeedUp);
//		Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 48, 24);
//		path.addSegment(seg1);
//		Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, 30), new Point(60, 30), 24);
//		path.addSegment(seg2);
//		Segment seg3 = new LineSegment(new Point(90, 30), new Point(90, 80), 48, 24);
//		path.addSegment(seg3);
//		Segment seg4 = new ArcSegment(new Point(90, 80), new Point(110, 100), new Point(110, 80), 24);
//		path.addSegment(seg4);
//		Segment seg5 = new LineSegment(new Point(110, 100), new Point(150, 100), 48, 0);
//		path.addSegment(seg5);
        
//		Segment seg1 = new LineSegment(new Point(0, 0), new Point(90, 0), 80, 60);
//		path.addSegment(seg1);
//		Segment seg2 = new ArcSegment(new Point(90, 0), new Point(140, -50), new Point(90, -50), 60, 0);
//		path.addSegment(seg2);
        
        // Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 80, 60);
        // path.addSegment(seg1);
        // Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, 30), new Point(60, 30), 60, 60);
        // path.addSegment(seg2);
        // Segment seg3 = new LineSegment(new Point(90, 30), new Point(90, 200), 80, 80);
        // path.addSegment(seg3);
        // Segment seg4 = new ArcSegment(new Point(90, 200), new Point(50, 240), new Point(50, 200), 60);
        // path.addSegment(seg4);
        // Segment seg5 = new LineSegment(new Point(50, 240), new Point(0, 240), 60, 0);
        // path.addSegment(seg5);
        
//		Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 60, 60);
//		path.addSegment(seg1);
//		Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, -30), new Point(60, -30), 60);
//		path.addSegment(seg2);
//		Segment seg3 = new ArcSegment(new Point(90, -30), new Point(120, -60), new Point(120, -30), 60);
//		path.addSegment(seg3);
//		Segment seg4 = new LineSegment(new Point(120, -60), new Point(170, -60), 60, 0);
//		path.addSegment(seg4);
        
        // Path path = new Path(true, true, Constants.kMaxAccelSpeedUp);
        // Segment seg1 = new LineSegment(new Point(170, -60), new Point(120, -60), 60);
        // path.addSegment(seg1);
        // Segment seg2 = new ArcSegment(new Point(120, -60), new Point(90, -30), new Point(120, -30), 60);
        // path.addSegment(seg2);
        // Segment seg3 = new ArcSegment(new Point(90, -30), new Point(60, 0), new Point(60, -30), 60);
        // path.addSegment(seg3);
        // Segment seg4 = new LineSegment(new Point(60, 0), new Point(0, 0), 60, 0);
        // path.addSegment(seg4);

        // Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 30, 30);
        // path.addSegment(seg1);
        // Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, -30), new Point(60, -30), 30);
        // path.addSegment(seg2);
        // Segment seg3 = new ArcSegment(new Point(90, -30), new Point(120, -60), new Point(120, -30), 30);
        // path.addSegment(seg3);
        // Segment seg4 = new LineSegment(new Point(120, -60), new Point(170, -60), 30, 0);
        // path.addSegment(seg4);

        Path path = AutoPathHelper.getPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_2);

        RobotPos currentPos = new RobotPos(path.getCurrentSegment().getStartPoint(), 0, 0,0);
        
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
            System.out.println(currentPos);
        }
        System.out.println("Estimate time Required " + pathTime + " sec");
        
    }

}
