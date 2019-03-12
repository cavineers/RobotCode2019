package frc.lib.pathPursuit;

import java.util.ArrayList;

import frc.lib.RobotPos;
import frc.lib.pathPursuit.ArcSegment.TURN;
import frc.robot.Constants;

public class Path {
    ArrayList<Segment> segmentList = new ArrayList<Segment>();
    boolean didFinish;
    boolean isReversed;
    double maxAccel;
    Lookahead lookahead;
    
    public VelocityManager manager;
    
    public Path(boolean isDebug, boolean isReversed, double maxAccel, Lookahead lookahead) {
        didFinish = false;
        manager = new VelocityManager(isDebug, isReversed, maxAccel, lookahead);
        this.lookahead = lookahead;
    }

    public Path(boolean isDebug, boolean isReversed, double maxAccel) {
        this(isDebug, false, Constants.kMaxAccelSpeedUp, new Lookahead(Constants.kMinLookAhead, Constants.kMaxLookAhead, Constants.kMinLookAheadSpeed, Constants.kMaxLookAheadSpeed));
    }
    
    public Path() {
        this(false, false, Constants.kMaxAccelSpeedUp);
    }
    
    /**
     * adds a segment to the path
     * 
     * @param segment: the segment 
     */
    public void addSegment(Segment segment) {
        segmentList.add(segment);
    }
    
    /**
     * updates which segment of the path the robot is currently driving on and creates a
     * RobotCmd containing left and right wheel velocities to make the robot stay on the path
     * 
     * @param state: the current position + velocity of the robot
     */
    public RobotCmd update(RobotPos robotPos) {
        Segment currentSegment = this.getCurrentSegment();
        Point lookaheadPt = currentSegment.getLookaheadPoint(robotPos.position, this.lookahead.getLookaheadForSpeed(robotPos.getVelocity()));
        // System.out.println(robotPos + "," + lookaheadPt);
        if (Point.getDistance(currentSegment.getEndPoint(), lookaheadPt) < Constants.kPathPursuitTolerance) {
            // if the current robot position is within tol of the end point, move on to the next segment
            // or say that the robot is done following the path
            if (this.canMoveOnToNextSegment()) {
                this.moveOnToNextSegment();
            } else { 
                //can't move onto the next segment, but it is within tol, so 
                if (Point.getDistance(robotPos.position, currentSegment.getEndPoint()) < Constants.kPathPursuitFinishTolerance && Math.abs(robotPos.getVelocity()) < 2) {
                    this.didFinish = true;
                }

                //stop the target point from becoming behind the robot
                if (Point.getDistance(currentSegment.getEndPoint(), robotPos.position) < Constants.kStopSteeringDistance) {
                    manager.freezeHeading(robotPos.heading);
                }
            }
        }

        return manager.getVelCmd(currentSegment, robotPos, lookaheadPt);
    }
    
    /*
     * returns if the robot can continue on to another segment or if it is at the end of a path
     */
    public boolean canMoveOnToNextSegment() {
        return segmentList.size() > 1;
    }
    
    /*
     * moves the robot onto the next segment of the path
     */
    private void moveOnToNextSegment() {
        segmentList.remove(0);
    }
    
    /*
     * returns the segment of the path the robot is driving on
     */
    public Segment getCurrentSegment() {
        return segmentList.get(0);
    }
    
    /*
     * returns if the robot reached the end of the final segment of the path
     */
    public boolean isFinished() {
        return didFinish;
    }

    /**
     * Mirrors the given path around the horizontal (long way) center line of the field
     * 
     * @param path a path to be mirrored
     * @return a mirrored path
     */
    public static Path mirror(Path path) {
        Path mirroredPath = new Path(false, path.isReversed, Constants.kMaxAccelSpeedUp);
        for (Segment seg : path.segmentList) {
            //iterate over all segments in a path, mirroring it around the centerline of the field
            if (seg instanceof ArcSegment) {
                ArcSegment arc = (ArcSegment) seg;
                Point mirroredStart = new Point(arc.getStartPoint().getX(), Constants.kFieldWidth - arc.getStartPoint().getY());
                Point mirroredEnd = new Point(arc.getEndPoint().getX(), Constants.kFieldWidth - arc.getEndPoint().getY());
                Point mirroredCenter = new Point(arc.getCenterPoint().getX(), Constants.kFieldWidth - arc.getCenterPoint().getY());
                
                //mirroring flips the turn types
                TURN newTurnType;
                if (arc.turnType == TURN.RIGHT) {
                    newTurnType = TURN.LEFT;
                } else {
                    newTurnType = TURN.RIGHT;
                }
                mirroredPath.addSegment(new ArcSegment(mirroredStart, mirroredEnd, mirroredCenter, arc.maxVel, arc.endVel, newTurnType));
            } else if (seg instanceof LineSegment) {
                LineSegment line = (LineSegment) seg;
                Point mirroredStart = new Point(line.getStartPoint().getX(), Constants.kFieldWidth - line.getStartPoint().getY());
                Point mirroredEnd = new Point(line.getEndPoint().getX(), Constants.kFieldWidth - line.getEndPoint().getY());
                mirroredPath.addSegment(new LineSegment(mirroredStart, mirroredEnd, line.maxVel, line.endVel));
            }
        }
        return mirroredPath;
    }

    @Override
    public String toString() {
        String s = "";
        for (Segment seg : segmentList) {
            if (seg instanceof ArcSegment) {
                ArcSegment arc = (ArcSegment) seg;
                System.out.println(arc.getStartPoint() + " " + arc.getEndPoint() + " " + arc.getCenterPoint() + " " + arc.smallSide);
            } else {
                LineSegment lin = (LineSegment) seg;
                System.out.println(lin.getStartPoint() + " " + lin.getEndPoint());
            }
            System.out.println(seg);
        }
        return s;
    }
    
}
