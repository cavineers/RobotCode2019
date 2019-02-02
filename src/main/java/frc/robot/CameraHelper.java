package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.TargetUpdate;

/**
 * A class which handles the network tables communcation between the roborio and raspberry pi vision coprocessors
 * 
 * network table channels:
 * 
 * Raspberry Pi status channels:
 * CamOnline - a boolean sent from the raspberry pi to the roborio indicating that it is online
 * 
 * Clock Synchronization Channels:
 * CamReadyForSync - a boolean sent from the raspberry pi to the roborio indicating if it is ready to synchroinze clocks
 * SuccessfulSync - a boolean sent from the raspberry pi to the roborio indicating if the pi successfully synchronized its clock
 * AttemptingToSync - a boolean sent from the roborio to the raspberry pi describing if it is trying to synchronize its clock with the pi
 * RioTime - the system time of the roborio sent from the roborio to the raspberry pi when a clock sync is being performed; otherwise it is -1
 * 
 * Target Data channels:
 * TargetUpdate - a string describing new target information from a raspberry pi in the following format:
 *                headingX,headingY,headingZ, targetX,targetY,targetZ, cameraX,cameraY,cameraZ, imageNum,timestamp
 */
public class CameraHelper {
    String name = "";
    NetworkTable netTable;
    int lastUpdateNum = -1; // the number of the last targetPos update

    public CameraHelper(String name) {
        this.name = name;
        netTable = NetworkTableInstance.getDefault().getTable(name);
        netTable.getEntry("RioTime").setNumber(-1);
    }

    /**
     * Tells if the raspberry pi is connected to the rio
     * 
     * @return whether or not the pi is connected to the rio
     */
    public boolean isPiConnected() {
        return netTable.getEntry("CamOnline").getBoolean(false) && NetworkTableInstance.getDefault().getConnections().length > 0;
    }

    /**
     * Tells if the pi is ready to sync slocks with the rio
     * 
     * @return whether or not the raspberry pi is ready for a clock synchonization with the rio
     */
    private boolean isPiReadyForClockSync() {
        return netTable.getEntry("CamReadyForSync").getBoolean(false);
    }
    
    /**
     * Tells if the raspberry pi has successfully synchronized clocks with the rio
     * 
     * @return if the raspberry pi has a update
     */
    private boolean didSuccessfullyClockSync() {
        return netTable.getEntry("SuccessfulSync").getBoolean(false);
    }
    /**
     * Tells if the rio should attempt to synchronze its clocks with the pi
     * 
     * @return if the rio should attempt to synchronze its clocks with the pi
     */
    public boolean shouldSyncClocks() {
        return this.isPiConnected() && this.isPiReadyForClockSync() && !this.didSuccessfullyClockSync();
    }

    /**
     * Starts the clock synchronization procedure between the rio and the pi
     */
    public void startClockSync() {
        if (this.shouldSyncClocks()) {
            System.out.println("Attempting to sync clocks");
            // the camera is connected and ready for a clock sync, and has not yet synchronzied
            netTable.getEntry("AttemptingToSync").setBoolean(true);
            // run clock synchronization in a seperate thread so that the main thread is not stopped for a long time while updating
            Thread clockSync = new Thread() {
                public void run() {
                    while (shouldSyncClocks()) {
                        // while the raspberry pi has not grabbed an update, update the riotime update as fast as possible
                        netTable.getEntry("RioTime").setNumber(Robot.getCurrentTime());
                        NetworkTableInstance.getDefault().flush();
                    }
                    netTable.getEntry("AttemptingToSync").setBoolean(false);
                    netTable.getEntry("RioTime").setNumber(-1);
                }  
            };

            clockSync.start();
            this.lastUpdateNum = -1;
        }
    }

    /**
     * Get the latest update from the raspberry pi, or null if there is no new update
     * 
     * @return a TargetPos describing the update if there was a new update, or null if there was no new update since last checked
     */
    public TargetUpdate getUpdate() {
        String updateString = netTable.getEntry("TargetUpdates").getString(""); //format: headingX, headingY, headingZ, targetX,targetY,targetZ,cameraX,cameraY,cameraZ,imageNum,timestamp
        
        System.out.println(updateString);
        if (updateString.isEmpty()) {
            System.out.println("EMPTY!!");
            return null; // no update available
        }
        // split the update string into its respective individual values
        String[] updateArr = updateString.split(",");

        if (updateArr.length != 11) {
            System.out.println("INVALID LENGTH!!");
            return null; // invalid update format
        }

        // extract numerical information from the string
        double headingX = Double.parseDouble(updateArr[0]);
        double headingY = Double.parseDouble(updateArr[1]);
        double headingZ = Double.parseDouble(updateArr[2]);

        double targetX = Double.parseDouble(updateArr[3]);
        double targetY = Double.parseDouble(updateArr[4]);
        double targetZ = Double.parseDouble(updateArr[5]);
        
        double cameraX = Double.parseDouble(updateArr[6]);
        double cameraY = Double.parseDouble(updateArr[7]);
        double cameraZ = Double.parseDouble(updateArr[8]);

        int imageNum = Integer.parseInt(updateArr[9]);
        double timestamp = Double.parseDouble(updateArr[10]);

        if (this.lastUpdateNum >= imageNum) {
            System.out.println("LAST IMG NUM!!");
            return null; // there is no new update available
        }

        this.lastUpdateNum = imageNum;

        return new TargetUpdate(headingX, headingY, headingZ, targetX, targetY, targetZ, cameraX, cameraY, cameraZ, imageNum, timestamp);
    }


}