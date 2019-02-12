package frc.robot;

import frc.robot.dashcomponents.*;
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
    private VideoStream fisheye;
    private String exportData;
    private NetworkTable netTable;
    private UpdatingText matchNum;

    DankDash() {
        // Nettables
        this.netTable = NetworkTableInstance.getDefault().getTable("DankDash");

        // fisheye video stream
        fisheye = new VideoStream();
        fisheye.setPath("http://10.45.41.5:8080?action=stream");
        fisheye.setXPos(20);
        fisheye.setYPos(50);
        fisheye.setZPos(1);
        fisheye.setWidth("100px");
        fisheye.setHeight("100px");
        System.out.println(fisheye.getJson(false));

        // // heartbeat
        // heartbeat = new Heartbeat("DankDash/heartbeat");
        // heartbeat.setXPos(600);
        // heartbeat.setYPos(800);
        // heartbeat.setZPos(2);
        // System.out.println(heartbeat.getJson(false));

        // Match Number
        matchNum = new UpdatingText("/FMSInfo/MatchNumber");
        matchNum.setXPos(40);
        matchNum.setYPos(40);
        matchNum.setZPos(1);
        matchNum.setTextColor("white");
    }

    public void export() {
        exportData = "["+fisheye.getJson(false)+"]";
        netTable.getEntry("ProfileData").setString(exportData);
        netTable.getEntry("ProfileName").setString("Test Chassis");
    }
}