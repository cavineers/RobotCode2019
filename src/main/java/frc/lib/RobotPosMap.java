package frc.lib;

import java.util.ArrayList;

import frc.lib.RobotPosUpdate.UpdateType;
import frc.lib.pathPursuit.Point;
import frc.robot.Robot;

public class RobotPosMap {
    // map goes from new updates (small indexes; larger timestamps) to old updates (large indexes; smaller timestamps)
    ArrayList<RobotPosUpdate> map = new ArrayList<>();

    double maxListSize = 0;

    boolean isBased = false;

    public RobotPosMap(int maxListSize) {
        this.maxListSize = maxListSize;
        isBased = false;
    }

    public RobotPosMap(int maxListSize, RobotPosUpdate base) {
        this.maxListSize = maxListSize;
        isBased = true;
        map.add(base);
    }
    
    /**
     * Add an update from wheel odometry
     * 
     * @param dx the change in x calculated from wheel odometry
     * @param dy the change in y calculated from wheel odometry
     * @param timestamp
     */
    public void addWheelUpdate(double dx, double dy, double timestamp) {
        this.addUpdate(new RobotPosUpdate(dx, dy, timestamp, UpdateType.WHEEL));
    }

    /**
     * Add an update from vision processing
     * 
     * @param x field relative xPos of the robot from the camera
     * @param y field relative yPos of the robot from the camera
     * @param timestamp time in seconds when the picture was taken
     */
    public void addCameraUpdate(double x, double y, double timestamp) {
        this.addUpdate(new RobotPosUpdate(x, y, timestamp, UpdateType.CAMERA));
    }

    /**
     * 
     * @param update
     */
    private void addUpdate(RobotPosUpdate update) {
        boolean foundHome = false;
        // starting from the front (lowest index / newest timestamps) of the array, look for an element
        // older than the new update, and then put the new update in front of the older one
        for (int i = 0; i < map.size(); i++) {
            if (map.get(i).getTimestamp() < update.getTimestamp()) {
                foundHome = true;

                // if the update is absolute (camera update), run an outlier check to make sure it isn't a crazy result,
                // if it fails, throw it out; if it passes, make it the new base of position calculations
                if (update.isAbsolute()) {
                    if (this.passesOutlierCheck(update)) {
                        this.rebaseMap(update);
                    } else {
                        return; //don't do anything
                    }
                } else {
                    // if the element's timestamp is older than the update's add it to the array at the old element's index
                    map.add(i, update);
                }
                
                break; // stop looking for older timestamps
            }
            // otherwise keep looking through the array for an older update
        }

        if (!foundHome && !update.isAbsolute()) {
            // there are no elements in the list older than the update, so add this one at the end
            map.add(map.size(), update);
        }

        while (map.size() > maxListSize) {
            // the list is larger than the max list size, consolidate the list

            // the base should be the last element in the list
            RobotPosUpdate oldBase = map.get(map.size() - 1);
            // remove the old base from the list
            map.remove(map.size()-1);
            // the new base of the list has the data from the previous update
            map.set(map.size()-1, oldBase.consolidateBaseWithUpdate(map.get(map.size()-1)));
        }
    }
    
    /**
     * Determine if an absolute update is an outlier, used for camera updates to ensure that no crazy
     * positions are made to be the base of positional estimations
     * 
     * @param update an absolute update that should be checked for being an outlier
     * @return whether or not the given update is an outlier
     */
    private boolean passesOutlierCheck(RobotPosUpdate update) {
        return true; //TODO: check for outliers
    }
    
    /**
     * Get the index of an update with the given timestamp if one exists, or the elements with timestamps on either side of the
     * requested timestamp if no element has the requested timestamp.  Return an empty array if the timestamp cannot be added to
     * the map (update is too old or timestamp is ahead of current time).
     * 
     * @param timestamp
     * @return an array containing the index of the element with the given timestamp, or the elements on either side, or 0 if it is a 
     *         valid update newer than anything else in the map, or an empty array if the timestamp cannot be added to the map
     */
    public int[] getMapIndexForTimestamp(double timestamp) {
        // use a binary search implementation to find the element with the requested timestamp
        // or the closest elements to the requested timestamp
        int first = 0;
        int last = map.size() - 1;
        int mid;
        while (first <= last) {
            mid = ( first + last ) / 2;
            if (timestamp == map.get(mid).getTimestamp()) {
                // an element exists with the given timestamp - return its index in an array
                System.out.println(timestamp);
                System.out.println(map.get(mid).getTimestamp());
                return new int[] {mid};
            } else if (map.get(mid).getTimestamp() < timestamp) {
                last = mid - 1;
            } else {
                first = mid + 1;
            }
        }
        if (last >= 0 && first < map.size()) {
            //TODO: verify that this is in the order first, last
            return new int[] {first, last}; // no element exists with the requested timestamp, but there are elements on either side of it
        } else if (last >= 0){
            // no element exists within the requested timestamp, and the timestamp is the smallest (oldest) in the array
            System.out.println("ERROR: CANNOT ADD UPDATE TO POSITION MAP; UPDATE TOO OLD!");
            return new int[] {}; // return nothing because it is too old
        } else {
            // no element exists within the requested timestamp, and the timestamp is the largest (newest) in the array
            if (timestamp > Robot.getCurrentTime()) {
                // the timestamp is ahead of current system time; throw it out
                System.out.println("ERROR: CANNOT ADD UPDATE TO POSITION MAP; UPDATE TOO NEW!");
                return new int[] {};
            } else {
                // the timestamp is the largest (newest) one in the array, and its time is valid
                return new int[] {0}; // return the index of the first update
            }
        }
    }

    /**
     * Rebases the robotPosMap with a new base
     * 
     * @param update
     */
    private void rebaseMap(RobotPosUpdate update) {
        // the base should be the last element in the list
        RobotPosUpdate oldBase = map.get(map.size() - 1);
        // remove the old base from the list
        map.remove(map.size()-1);

        //get the index(es) that the update should replace
        int[] indexArr = this.getMapIndexForTimestamp(update.getTimestamp());
        if (indexArr.length == 0) {
            // the map cannot be rebased, revert the old base and stop]
            map.add(map.size(), oldBase); // add the old base to the end of the map
            return;
        }
        if (indexArr.length == 1) {
            // there is a whole number index that the base can replace
            int index = indexArr[0];

            // make the new update a base and insert it at the given index
            map.set(index, update.makeBase());

            // remove all elements after the new base
            while(map.size() - 1 != index) {
                map.remove(map.size() - 1);
            }
        }
        if (indexArr.length == 2) {
            // there is no whole number index that the base can replace, create a partial update representing
            // the movement in between the last update and the base
            RobotPosUpdate partialUpdate = RobotPosUpdate.createUpdateAtTime(map.get(indexArr[0]), map.get(indexArr[1]), update.getTimestamp());

            // replace the total wheel positional update with the partial update we just created
            map.set(indexArr[1], partialUpdate);

            // make the index before the partial update the new base
            map.set(indexArr[0], update.makeBase());

            // remove all elements after the new base
            while(map.size() - 1 != indexArr[0]) {
                map.remove(map.size() - 1);
            }
        }

    }

    /**
     * Get the map base of the current RobotPosMap, or null if the map is not based
     * @return map base or null
     */
    private RobotPosUpdate getMapBase() {
        if (!this.isBased) {
            return null;
        }
        return map.get(map.size() - 1);
    }

    public RobotPos getFieldRelativePosition() {
        if (!this.isBased) {
            return null;
        }

        RobotPosUpdate base = this.getMapBase();
        double xTot = base.getX();
        double yTot = base.getY();

        for (int i = 0; i < map.size() - 1; i++) {
            RobotPosUpdate rel = map.get(i);
            xTot += rel.getDx();
            yTot += rel.getDy();
        }

        return new RobotPos(new Point(xTot, yTot), 0, 0, 0);
    }

    public void printMap() {
        for (RobotPosUpdate update : map) {
            System.out.println(update);
        }
        
    }

}