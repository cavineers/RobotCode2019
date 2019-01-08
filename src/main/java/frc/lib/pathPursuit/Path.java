package frc.lib.pathPursuit;

import java.util.ArrayList;

import frc.lib.RobotPos;
import frc.robot.Constants;

public class Path {
	ArrayList<Segment> segmentList = new ArrayList<Segment>();
	boolean didFinish;
	boolean isReversed;
	double maxAccel;
	
	public VelocityManager manager;
	
	public Path(boolean isDebug, boolean isReversed, double maxAccel) {
		didFinish = false;
		manager = new VelocityManager(isDebug, isReversed);
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
//		Point currentPos = robotPos.position;
		Point lookaheadPt = currentSegment.getLookaheadPoint(robotPos.position, Constants.lookahead.getLookaheadForSpeed(robotPos.getVelocity()));
		
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
			}
			
			//stop the target point from becoming behind the robot
			if (Point.getDistance(currentSegment.getEndPoint(), robotPos.position) < Constants.kStopSteeringDistance) {
				manager.freezeHeading(robotPos.heading);
			}
		}
		
		return manager.getVelCmd(currentSegment, robotPos);
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
	public void moveOnToNextSegment() {
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
	
}
