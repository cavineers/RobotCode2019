package frc.lib;

import java.util.Random;

import frc.lib.pathPursuit.Point;

public class MathHelper {
    
    /**
     * Checks if two doubles are approximately equal (equal to 3 decimal places)
     * Used to circumvent floating point rounding errors from making == fail for
     * equal values
     * 
     * @param v1: the first value to be compared
     * @param v2: the second value to be compared
     * @return if v1 and v2 are equal to 3 decimal places
     */
    public static boolean areApproxEqual(double v1, double v2) {
        return Math.round(v1 * 1000) == Math.round(v2 * 1000);
    }
    
    /**
     * Checks if two doubles are equal or within a given tolerance of one another
     * 
     * @param v1: the first value to be compared
     * @param v2: the second value to be compared
     * @param tol: the tolerance
     * @return if v1 is within tol of v2
     */
    public static boolean areEqualWithinTol(double v1, double v2, double tol) {
        double diff = Math.abs(v1 - v2);
        return diff <= tol;
    }
    /**
     * Takes an angle in radians and returns that angle between -pi and pi
     * 
     * @param angle: the angle to be converted
     * @return an angle between -Pi and Pi
     */
    public static double angleToNegPiToPi(double angle) {
        return angle - Math.PI * 2 * Math.floor((angle + Math.PI) / (Math.PI * 2));
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
     * return the angle of the arc formed by a given start point, endpoint, and center point
     * 
     * @param startPoint the start point of arc
     * @param endPoint the end point of the arc
     * @param centerPoint the center point of the arc
     * @return the angle of the given arc
     */
    public static double getAngleForArc(Point startPoint, Point endPoint, double radius) {
        // get the three sides of the triangle
        double c = Point.getDistance(startPoint, endPoint);
        
        //solve for arc measure with law of cosines
        return Math.acos((c*c - radius*radius - radius*radius) / (-2 * radius * radius));
    }
    
    /**
     * return a random double between two values
     * @param min the minimum value for the random number
     * @param max the maximum value for the random number
     * @return a random double between min and max
     */
    public static double getRandomDouble(double min, double max) {
        Random random = new Random();
        return random.nextDouble() * (max - min) + min;
    }
}
