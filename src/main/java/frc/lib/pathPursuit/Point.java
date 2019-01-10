package frc.lib.pathPursuit;

import frc.lib.Vector;

public class Point {
	
	private double x = 0;
	private double y = 0;
	
	public Point(double xCord, double yCord) {
		this.x = xCord;
		this.y = yCord;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	/**
	 * Get the distance between the origin and the point
	 */
	public double getHypot() {
		return Math.hypot(this.getX(), this.getY());
	}
	
	@Override
	public String toString() {
		return this.getX() + ", " + this.getY();
	}
	
	/**
	 * Take the cross product between two points
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static double cross(Point a, Point b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }
	
	/**
	 * Create a point which represents the delta between
	 * two points
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static Point getDelta(Point a, Point b) {
		return new Point(b.getX() - a.getX(), b.getY() - a.getY());
	}
	
	/**
	 * Add two points together, then return the sum
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static Point addPoints(Point a, Point b) {
		return new Point(a.getX() + b.getX(), a.getY() + b.getY());
	}
	
	/**
	 * Subtract point b from point a, then return the difference
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static Point subtractPoints(Point a, Point b) {
		return new Point(a.getX() - b.getX(), a.getY() - b.getY());
	}
	
	/**
	 * return the distance between two points
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static double getDistance(Point a, Point b) {
		return Math.hypot(b.getX() - a.getX(), b.getY() - a.getY());
	}
	
	/**
	 * return the angle the robot needs to get from its current position to a desired position
	 * (angle between two points and x-axis)
	 * 
	 * @param currentPos the current position of the robot
	 * @param desiredPos the point we want to be at
	 */
	public static double getAngleNeeded(Point currentPos, Point desiredPos) {
		double dx = desiredPos.getX() - currentPos.getX() ;
		double dy = desiredPos.getY() - currentPos.getY();
		return Math.atan2(dy, dx);
	}
	
	/**
	 * return the angle of the arc formed by a given start point, endpoint, and center point
	 * 
	 * @param startPoint the start point of arc
	 * @param endPoint the end point of the arc
	 * @param centerPoint the center point of the arc
	 * @return the angle of the given arc
	 */
	public static double getAngleForArc(Point startPoint, Point endPoint, Point centerPoint) {
		// get the three sides of the triangle
		double a = Point.getDistance(centerPoint, startPoint);
		double b = Point.getDistance(centerPoint, endPoint);
		double c = Point.getDistance(startPoint, endPoint);
		
		//solve for arc measure with law of cosines
		return Math.acos((c*c - a*a - b*b) / (-2 * a * b));
	}
	
	/**
	 * returns the side length opposite to thetaC in triangle ABC
	 */
	public static double getOppSideLength(double a, double b, double thetaC) {
		//solve for side length with law of cosines
		return Math.sqrt(a*a + b*b - 2 * a * b * Math.cos(thetaC));
	}
	
	/**
	 * Converts the reference frame of p1 such that p2 is the origin and rotates it by theta
	 * 
	 * @param p1: the point whose origin should be changed
	 * @param p2: new origin
	 * @param heading: the way that p1 will be rotated
	 * @return: p1 relative to p2 rotated by angle theta
	 */
	public static Point getP1RelativeToP2(Point p1, Point p2, double theta) {
		double angle = theta;// + Math.PI / 2;  //Convert to a heading zeroed at the x axis (?)
		double y = (p1.getX() - p2.getX()) * Math.cos(angle) + (p1.getY() - p2.getY()) * Math.sin(angle);
		double x = -1 * (p1.getX() - p2.getX()) * Math.sin(angle) + (p1.getY() - p2.getY()) * Math.cos(angle);
		
		return new Point(x, y);
	}

	/**
	 * gets a vector with the same values as the current point
	 * @return a vector representing the point
	 */
	public Vector getVector() {
		return new Vector(this.x, this.y);
	}
}
