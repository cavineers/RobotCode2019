package frc.lib;

import frc.lib.pathPursuit.Point;

public class Vector {
    double dx;
    double dy;
    
    public Vector(double dx, double dy) {
        this.dx = dx;
        this.dy = dy;
    }

    public double getDx() {
        return dx;
    }

    public double getDy() {
        return dy;
    }

    /**
     * Rotates the vector by the given angle
     * @param angleRad the angle to rotate the vector by
     * @return a vector rotated by angleRad
     */
    public Vector rotate(double angleRad) {
        double dxRot = this.dx * Math.cos(angleRad) - this.dy * Math.sin(angleRad);
        double dyRot = this.dx * Math.sin(angleRad) + this.dy * Math.cos(angleRad);

        return new Vector(dxRot, dyRot);
    }

    /**
     * Subtracts Vector v2 from Vector v1
     * 
     * @param v1 the number v2 will be subtracted from
     * @param v2 the number to subtract from v1
     * @return the result of v1 - v2
     */
    public static Vector subtract(Vector v1, Vector v2) {
        return new Vector(v1.getDx() - v2.getDx(), v1.getDy() - v2.getDy());
    }

    /**
     * Adds the two given vectors together
     * 
     * @param v1 the first vector to sum
     * @param v2 the second vector to sum
     * @return the sum of the two vectors
     */
    public static Vector add(Vector v1, Vector v2) {
        return new Vector(v1.getDx() + v2.getDx(), v1.getDy() + v2.getDy());
    }

    /**
     * Gets a point whose x and y points are the dx and dy of the current vector
     * 
     * @return a point representing the current vector
     */
    public Point getPoint() {
        return new Point(dx, dy);
    }
}