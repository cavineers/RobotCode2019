package frc.robot;

// import frc.robot.dashcomponents.*;
import edu.wpi.first.networktables.*;

// ===== Available Components =====
//
// Button
// Checkbox
// Container
// Heartbeat
// Image
// Refresh button
// Static Text
// Updating Text
// Video stream

public class DankDash {
    private NetworkTable netTable;
    private String profileName;
    private String profileLocation;

    DankDash() {
        // Nettables
        this.netTable = NetworkTableInstance.getDefault().getTable("DankDash");
    }

    public void setProfileName(String profileName) {
        this.profileName = profileName;
    }

    public String getProfileName() {
        return this.profileName;
    }

    public void setProfileLocation(String profileLocation) {
        this.profileLocation = profileLocation;
    }

    public String getProfileLocation() {
        return this.profileLocation;
    }

    public void export() {
        netTable.getEntry("ProfileData").setString(this.profileLocation);
        netTable.getEntry("ProfileName").setString(this.profileName);
    }

    public void sendDash(String id, String data){
        netTable.getEntry(id).setString(data);
    }
}