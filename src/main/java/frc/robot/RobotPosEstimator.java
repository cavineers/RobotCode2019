package frc.robot;

import java.util.concurrent.locks.ReentrantLock;

import frc.lib.MathHelper;
import frc.lib.pathPursuit.Point;
import frc.robot.Robot;

public class RobotPosEstimator {

	double xPos = 0;
	double yPos = 0;
	double heading = 0;
	double rWheelTravel = 0;
	double lWheelTravel = 0;
	boolean isLocked = false;

	ReentrantLock mutex = new ReentrantLock();
	
	Thread thread;
	
	private volatile boolean running = false;

	public RobotPosEstimator(double x, double y, double h, double rWT, double lWT) {
		xPos = x; // in inches
		yPos = y; // in inches
		heading = h; // in radians
		rWheelTravel = rWT; // in rotations
		lWheelTravel = lWT; // in rotations
	}

	public void updatePos(double newRTravel, double newLTravel, double newHeading) {
		double dR = newRTravel - rWheelTravel;
		double dL = newLTravel - lWheelTravel;
		double dH = rotate(getInverseOfAngle(heading), newHeading);
		double epsilon = 0.0000001; // threshold between circular and linear estimations
		if (dH > epsilon) { // estimate change in position using constant radius math
			double radius = (dR + dL) / (2 * dH);
			double mag = 2 * radius * Math.sin(dH / 2);
			double XMag = mag * Math.cos(newHeading);
			double YMag = mag * Math.sin(newHeading);
			xPos += XMag;
			yPos += YMag;
		} else { // estimate change in position using linear distance between points
			double mag = (dR + dL) / 2;
			yPos += Math.sin(newHeading) * mag;
			xPos += Math.cos(newHeading) * mag;
		}
		heading = newHeading;
		rWheelTravel = newRTravel;
		lWheelTravel = newLTravel;
	}

	/*
	 * Effectively undoes the rotation by an angle; If a line segment is rotated by
	 * the angle and then its inverse, it will have the same heading as before being
	 * rotated
	 */
	public double getInverseOfAngle(double angleRad) {
		return Math.atan2((-1 * Math.sin(angleRad)), Math.cos(angleRad));
	}

	/*
	 * Rotates an a1 by a2 using a rotation matrix
	 */
	public double rotate(double a1, double a2) {
		return Math.atan2(Math.cos(a1) * Math.sin(a2) + Math.sin(a1) * Math.cos(a2),
				Math.cos(a1) * Math.cos(a2) - Math.sin(a1) * Math.sin(a2));
	}

	/*
	 * Get the net distance traveled by the right wheels of the robot
	 */
	public double getRightWheelTravel() {
		return rWheelTravel; // in inches
	}

	/*
	 * Get the net distance traveled by the left wheels of the robot
	 */
	public double getLeftWheelTravel() {
		return lWheelTravel; // in inches
	}

	/*
	 * Get the heading of the robot (from the gyroscope at the time of the last
	 * update)
	 */
	public double getHeading() {
		return MathHelper.angleToNegPiToPi(Math.toRadians(Robot.gyro.getAngle())); // in radians
	}

	public double getRightWheel() {
		return Robot.drivetrain.getRightPos();
	}

	public double getLeftWheel() {
		return Robot.drivetrain.getLeftPos();
	}

	public double getX() {
		mutex.lock();
		try {
			return xPos;
		} finally {
			mutex.unlock();
		}
	}

	public double getY() {
		mutex.lock();
		try {
			return yPos;
		} finally {
			mutex.unlock();
		}
	}
	
	public Point getPosition() {
		mutex.lock();
		try {
			return new Point(this.xPos, this.yPos);
		} finally {
			mutex.unlock();
		}
	}

	public void start() {
		this.running = true;
		thread = new Thread(() -> {
			while (this.running) {

				mutex.lock();
				// when locked, continously update position
				try {
					this.updatePos(getRightWheel(), getLeftWheel(), getHeading());
				} finally {
					mutex.unlock();
				}

			}
		});
		thread.start();
	}
	
	public void end() {
		this.running = false;
	}
	
	public void zero() {
		mutex.lock();
		try {
			xPos = 0;
			yPos = 0;
		} finally {
			mutex.unlock();
		}
	}

}
