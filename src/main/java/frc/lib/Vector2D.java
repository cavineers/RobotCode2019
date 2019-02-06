package frc.lib;

import frc.lib.pathPursuit.Point;

public class Vector2D {
    double dx;
    double dy;
    
    public Vector2D(double dx, double dy) {
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
    public Vector2D rotate(double angleRad) {
        double dxRot = this.dx * Math.cos(angleRad) - this.dy * Math.sin(angleRad);
        double dyRot = this.dx * Math.sin(angleRad) + this.dy * Math.cos(angleRad);

        return new Vector2D(dxRot, dyRot);
    }

    /**
     * Subtracts Vector v2 from Vector v1
     * 
     * @param v1 the number v2 will be subtracted from
     * @param v2 the number to subtract from v1
     * @return the result of v1 - v2
     */
    public static Vector2D subtract(Vector2D v1, Vector2D v2) {
        return new Vector2D(v1.getDx() - v2.getDx(), v1.getDy() - v2.getDy());
    }

    /**
     * Adds the two given vectors together
     * 
     * @param v1 the first vector to sum
     * @param v2 the second vector to sum
     * @return the sum of the two vectors
     */
    public static Vector2D add(Vector2D v1, Vector2D v2) {
        return new Vector2D(v1.getDx() + v2.getDx(), v1.getDy() + v2.getDy());
    }

    /**
     * Gets a point whose x and y points are the dx and dy of the current vector
     * 
     * @return a point representing the current vector
     */
    public Point getPoint() {
        return new Point(dx, dy);
    }

    /**
     * Gets the angle between the vector and the x-axis
     * 
     * @return the angle between the vector and the x-axis
     */
    public double getAngle() {
        return Math.atan2(dy, dx);
    }

    /**
     * Gets the cross product of two vectors
     * @param v1 the first vector
     * @param v2 the second vector
     * @return the cross product of v1 and v2
     */
    public static double getCrossProduct(Vector2D v1, Vector2D v2) {
        return v1.dx * v2.dy - v1.dy * v2.dx;
    }

    @Override
    public String toString() {
        return this.dx + ", " + this.dy; 
    }
}