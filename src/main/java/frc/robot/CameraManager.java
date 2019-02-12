package frc.robot;

import frc.lib.TargetUpdate;
//TODO: outlier check
public class CameraManager {
    TargetUpdate latestUpdate = null;

    public void checkForCamUpdates() {
        latestUpdate = Robot.reflectiveTapeCamera.getUpdate();
    }

    public boolean hasValidCameraUpdate() {
        if (latestUpdate == null || (latestUpdate.getTimestamp() > Robot.getCurrentTime() - 3  && latestUpdate.getTimestamp() < Robot.getCurrentTime())) {
            return false; //the latest update is null or is not within 3 seconds of latest time
        }
        return true; // the update is good
    }

    public TargetUpdate getLatestUpdate() {
        return latestUpdate;
    }

}