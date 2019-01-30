package frc.lib;

import frc.lib.pathPursuit.Point;

public class TargetUpdate {
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
     * 
     * @param targX x position of the target relative to the robot in target coords
     * @param targY y position of the target relative to the robot in target coords
     * @param targZ z position of the target relative to the robot in target coords
     
     * @param camX x position of the target relative to the robot in camera coords
     * @param camY y position of the target relative to the robot in camera coords
     * @param camZ z position of the target relative to the robot in camera coords
     * 
     * @param updateNum the positive whole number describing the number of the update
     * @param timestamp time the target was aquired
     */
    public TargetUpdate(double targX, double targY, double targZ, double camX, double camY, double camZ, int updateNum, double timestamp) {
        this.targX = targX;
        this.targY = targY;
        this.targZ = targZ;

        this.camX = camX;
        this.camY = camY;
        this.camZ = camZ;

        this.updateNum = updateNum;
        this.timestamp = timestamp;
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
}