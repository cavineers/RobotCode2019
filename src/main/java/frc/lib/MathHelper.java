package frc.lib;

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
}
