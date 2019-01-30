package frc.lib;

import frc.lib.pathPursuit.Point;

public class Vector3D {
    double dx;
    double dy;
    double dz;
    
    public Vector3D(double dx, double dy, double dz) {
        this.dx = dx;
        this.dy = dy;
        this.dz = dz;
    }

    /**
     * Gets the x component of the vector
     * 
     * @return the x component of the given vector
     */
    public double getDx() {
        return dx;
    }

    /**
     * Gets the y component of the vector
     * 
     * @return the y component of the given vector
     */
    public double getDy() {
        return dy;
    }

    /**
     * Gets the z component of the vector
     * 
     * @return the z component of the given vector
     */
    public double getDz() {
        return dz;
    }

    /**
     * Rotates the vector by the given angle around the X axis
     * @param angleRad the angle to rotate the vector by
     * @return a vector rotated by angleRad
     */
    public Vector3D rotateXAxis(double angleRad) {
        return new Vector3D(dx, dy * Math.cos(angleRad) + dz * Math.sin(angleRad), - dy * Math.sin(angleRad) + dz * Math.cos(angleRad));
    }

    /**
     * Rotates the vector by the given angle around the Y axis
     * @param angleRad the angle to rotate the vector by
     * @return a vector rotated by angleRad
     */
    public Vector3D rotateYAxis(double angleRad) {
        return new Vector3D(dx * Math.cos(angleRad) - dz * Math.sin(angleRad), dy, dx * Math.sin(angleRad) + dz * Math.cos(angleRad));
    }

    /**
     * Rotates the vector by the given angle around the Z axis
     * @param angleRad the angle to rotate the vector by
     * @return a vector rotated by angleRad
     */
    public Vector3D rotateZAxis(double angleRad) {
        return new Vector3D(dx * Math.cos(angleRad) + dy * Math.sin(angleRad), -dx * Math.sin(angleRad) + dy * Math.cos(angleRad), dz);
    }

    /**
     * Subtracts Vector v2 from Vector v1
     * 
     * @param v1 the number v2 will be subtracted from
     * @param v2 the number to subtract from v1
     * @return the result of v1 - v2
     */
    public static Vector3D subtract(Vector3D v1, Vector3D v2) {
        return new Vector3D(v1.getDx() - v2.getDx(), v1.getDy() - v2.getDy(), v1.getDz() - v2.getDz());
    }

    /**
     * Adds the two given vectors together
     * 
     * @param v1 the first vector to sum
     * @param v2 the second vector to sum
     * @return the sum of the two vectors
     */
    public static Vector3D add(Vector3D v1, Vector3D v2) {
        return new Vector3D(v1.getDx() + v2.getDx(), v1.getDy() + v2.getDy(), v1.getDz() + v2.getDz());
    }

    @Override
    public String toString() {
        return "[" + this.getDx() + "\t" + this.getDy() + "\t" + this.getDz() + "]";
    }

}