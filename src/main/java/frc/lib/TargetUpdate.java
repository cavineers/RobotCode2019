package frc.lib;

import frc.lib.pathPursuit.Point;

public class TargetUpdate {
    double headingX;
    double headingY;
    double headingZ;

    double targX;
    double targY;
    double targZ;

    double camX;
    double camY;
    double camZ;

    int updateNum;
    double timestamp;

    /**
     * A helper class dedicated to storing information about a target
     * @param headingX x component of a unit vector representing the heading of the target
     * @param headingY y component of a unit vector representing the heading of the target
     * @param headingZ z component of a unit vector representing the heading of the target
     * 
     * @param targX x position of the camera relative to the target in target coords
     * @param targY y position of the camera relative to the target in target coords
     * @param targZ z position of the camera relative to the target in target coords
     
     * @param camX x position of the target relative to the camera in camera coords
     * @param camY y position of the target relative to the camera in camera coords
     * @param camZ z position of the target relative to the camera in camera coords
     * 
     * @param updateNum the positive whole number describing the number of the update
     * @param timestamp time the target was aquired
     */
    public TargetUpdate(double headingX, double headingY, double headingZ, double targX, double targY, double targZ, double camX, double camY, double camZ, int updateNum, double timestamp) {
        this.headingX = headingX;
        this.headingY = headingY;
        this.headingZ = headingZ;

        this.targX = targX;
        this.targY = targY;
        this.targZ = targZ;

        this.camX = camX;
        this.camY = camY;
        this.camZ = camZ;

        this.updateNum = updateNum;
        this.timestamp = timestamp;
    }

    public Vector3D getHeadingVector() {
        return new Vector3D(headingX, headingY, headingZ);
    }

    public Vector3D getTargetVector() {
        return new Vector3D(targX, targY, targZ);
    }

    public Vector3D getCameraVector() {
        return new Vector3D(camX, camY, camZ);
    }

    public int updateNum() {
        return updateNum;
    }

    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public String toString() {
        return headingX + "," + headingY + "," + headingZ + targX + "," + targY + "," + targZ + "," + camX + "," + camY + "," + camZ + "," + updateNum + "," + timestamp;
    }
}