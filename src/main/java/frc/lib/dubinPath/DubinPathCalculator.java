package frc.lib.dubinPath;

import java.util.ArrayList;

import frc.lib.dubinPath.geometry.Geometry;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Lookahead;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.Segment;
import frc.robot.Constants;

public class DubinPathCalculator {

    public static DubinsPath getBestPath(Point start, double startHeading, Point end, double endHeading) {
        double radius = getBestTurnRadiusForPath(start, startHeading, end, endHeading);

        DubinsTrajectory traj = getTraj(start, startHeading, end, endHeading, radius);
        ArrayList<Segment> segList = traj.getSegmentList();
        
        Path path = new Path(false, false, Constants.kMaxTargetAccel,  new Lookahead(Constants.kMinLookAheadTargeting, Constants.kMinLookAheadTargeting, Constants.kMinLookAheadSpeedTargeting, Constants.kMaxLookAheadSpeedTargeting));
        for (Segment seg : segList) {
            path.addSegment(seg);
        }

        //add the straight line segment to the end of the path
        double x = end.getX() - Constants.kStraightLineDistance * Math.cos(endHeading);
        double y = end.getY() - Constants.kStraightLineDistance * Math.sin(endHeading);
        Point dubinEnd = new Point(x,y);
        path.addSegment(new LineSegment(dubinEnd, end, Constants.kMaxTargetSpeed, 0));

        //determine if the newly generated path is valid according to the constraints in Constants
        boolean isValid = true;
        if (radius < Constants.kMinRadiusTargeting) {
            isValid = false;
        }

        return new DubinsPath(path, radius, isValid);
    }

    private static DubinsTrajectory getTraj(Point start, double startHeading, Point end, double endHeading, double radius) {
        Dubins dubins = new Dubins();
        dubins.setMinTurnRadius(radius);
        DubinsTrajectory traj = null;

        double x = end.getX() - Constants.kStraightLineDistance * Math.cos(endHeading);
        double y = end.getY() - Constants.kStraightLineDistance * Math.sin(endHeading);
        Point dubinEnd = new Point(x,y);

        try {
            traj = dubins.getShortestPath(new Geometry.Pose2d(start.getX(), start.getY(), startHeading), new Geometry.Pose2d(dubinEnd.getX(), dubinEnd.getY(), endHeading));
        } catch (Exception e) {
            System.out.println(e);
            return null;
        }

        return traj;
    }

    private static double getBestTurnRadiusForPath(Point start, double startHeading, Point end, double endHeading) {

        DubinsTrajectory newTraj = getTraj(start, startHeading, end, endHeading, 0.5);

        String idealPathType = newTraj.type.name();

        double bestTurnRadius = 2;

        while (newTraj.type.name().equals(idealPathType)) {
            bestTurnRadius += 2;
            newTraj = getTraj(start, startHeading, end, endHeading, bestTurnRadius);
        } 

        return bestTurnRadius - 2;
    }
}