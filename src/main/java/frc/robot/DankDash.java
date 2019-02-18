package frc.robot;

// import frc.robot.dashcomponents.*;
import edu.wpi.first.networktables.*;
import frc.robot.commands.CheckEncoders;

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
    private CheckEncoders encoderCheck;

    DankDash() {
        // Nettables
        this.netTable = NetworkTableInstance.getDefault().getTable("DankDash");
    }

    public void setProfileName(String profileName) {
        this.profileName = profileName;
    }

    public void encoderCheck() {
        netTable.addEntryListener((table, key, entry, value, flags) -> {
            System.out.println("Key: "+key);
            if (key == "") {
                if (value.getBoolean()) {
                    encoderCheck = new CheckEncoders();
                    netTable.getEntry("").setBoolean(false);
                }
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
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