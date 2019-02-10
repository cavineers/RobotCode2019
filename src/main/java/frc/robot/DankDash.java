package frc.robot;

import frc.robot.dashcomponents.*;
import edu.wpi.first.networktables.*;

// ===== Available Components =====
//
// Container
// Heartbeat
// Image
// Refresh button
// Static Text
// Updating Text
// Video stream

public class DankDash {
    private VideoStream fisheye;
    private Heartbeat heartbeat;
    private String exportData;
    private NetworkTable netTable;

    DankDash() {
        // Nettables
        NetworkTable netTable = NetworkTableInstance.getDefault().getTable("DankDash");

        // fisheye video stream
        fisheye = new VideoStream();
        fisheye.setPath("http://10.45.41.5:8080?action=stream");
        fisheye.setXPos(20);
        fisheye.setYPos(50);
        fisheye.setZPos(1);
        System.out.println(fisheye.getJson(false));

        // heartbeat
        heartbeat = new Heartbeat("DankDash/heartbeat");
        heartbeat.setXPos(600);
        heartbeat.setYPos(800);
        heartbeat.setZPos(2);
        System.out.println(heartbeat.getJson(false));
    }

    public void export() {
        exportData = "{"+fisheye.getJson(true)+heartbeat.getJson(false)+"}";
        netTable.getEntry("profileData").setString(exportData);
    }
}