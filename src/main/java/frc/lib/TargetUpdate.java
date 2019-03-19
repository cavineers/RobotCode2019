package frc.lib;

import frc.lib.pathPursuit.Point;

public class TargetUpdate {
    double vhtcs_x;
    double vhtcs_y;
    double vhtcs_z;

    double vctts_x;
    double vctts_y;
    double vctts_z;

    double vtccs_x;
    double vtccs_y;
    double vtccs_z;

    int updateNum;
    double timestamp;

    /**
     * A helper class dedicated to storing information about a target
     * VECTOR NAMING SYSTEM (HEAD, TAIL, COORDINATE SYSTEM)
     * @param vhtcs_x x component target heading vector (head, target, camera system)
     * @param vhtcs_y y component target heading vector (head, target, camera system)
     * @param vhtcs_z z component target heading vector (head, target, camera system)
     * 
     * @param vctts_x x target heading (camera, target, target system)
     * @param vctts_y y target heading (camera, target, target system)
     * @param vctts_z z target heading (camera, target, target system)
     * 
     * @param vtccs_x x camera vector (target, camera, camera system)
     * @param vtccs_y y camera vector (target, camera, camera system)
     * @param vtccs_z z camera vector (target, camera, camera system)
     * 
     * @param updateNum the positive whole number describing the number of the update
     * @param timestamp time the target was aquired
     */
    public TargetUpdate(double vhtcs_x, double vhtcs_y, double vhtcs_z, double vctts_x, double vctts_y, double vctts_z, double vtccs_x, double vtccs_y, double vtccs_z, int updateNum, double timestamp) {
        this.vhtcs_x = vhtcs_x;
        this.vhtcs_y = vhtcs_y;
        this.vhtcs_z = vhtcs_z;

        this.vctts_x = vctts_x;
        this.vctts_y = vctts_y;
        this.vctts_z = vctts_z;

        this.vtccs_x = vtccs_x;
        this.vtccs_y = vtccs_y;
        this.vtccs_z = vtccs_z;

        this.updateNum = updateNum;
        this.timestamp = timestamp;
    }

    public Vector3D getVhtcs() {
        return new Vector3D(vhtcs_x, vhtcs_y, vhtcs_z);
    }

    public Vector3D getVctts() {
        return new Vector3D(vctts_x, vctts_y, vctts_z);
    }

    public Vector3D getVtccs() {
        return new Vector3D(vtccs_x, vtccs_y, vtccs_z);
    }

    public int updateNum() {
        return updateNum;
    }

    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public String toString() {
        return vhtcs_x + "," + vhtcs_y + "," + vhtcs_z + "," + vctts_x + "," + vctts_y + "," + vctts_z + "," + vtccs_x + "," + vtccs_y + "," + vtccs_z + "," + updateNum + "," + timestamp;
    }
}