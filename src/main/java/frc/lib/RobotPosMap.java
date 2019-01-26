package frc.lib;

import java.util.ArrayList;

import frc.lib.RobotPosUpdate.UpdateType;
import frc.lib.pathPursuit.Point;
import frc.robot.Constants;
import frc.robot.Robot;

public class RobotPosMap {
    // map goes from new updates (small indexes; larger timestamps) to old updates (large indexes; smaller timestamps)
    ArrayList<RobotPosUpdate> map = new ArrayList<RobotPosUpdate>();

    double maxListSize = 0;

    boolean isBased = false;

    boolean allowsCameraUpdates = true;

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
     * @param heading the heading when the updated was captured
     * @param timestamp the time when the update was captured
     */
    public void addWheelUpdate(double dx, double dy, double heading, double timestamp) {
        this.addUpdate(new RobotPosUpdate(dx, dy, heading, timestamp, UpdateType.WHEEL));
    }

    /**
     * Add an update from vision processing
     * 
     * @param x field relative xPos of the robot from the camera
     * @param y field relative yPos of the robot from the camera
     * @param timestamp time in seconds when the picture was taken
     */
    public void addCameraUpdate(double x, double y, double heading, double timestamp) {
        this.addUpdate(new RobotPosUpdate(x, y, heading, timestamp, UpdateType.CAMERA));
    }

    /**
     * Adds a positional update to the RobotPos map at the desired timestamp and colsolidates the 
     * map if its size is greater than the max specified size
     * 
     * @param update the latest robot pos update
     */
    private void addUpdate(RobotPosUpdate update) {
        if (update.type == UpdateType.CAMERA && !this.allowsCameraUpdates) {
            return;
        }

        boolean foundHome = false;
        // starting from the front (lowest index / newest timestamps) of the array, look for an element
        // older than the new update, and then put the new update in front of the older one
        for (int i = 0; i < map.size(); i++) {
            if (map.get(i).getTimestamp() < update.getTimestamp()) {
                foundHome = true;

                // if the update is absolute (camera update), run an outlier check to make sure it isn't a crazy result,
                // if it fails, throw it out; if it passes, make it the new base of position calculations
                if (update.isAbsolute()) {
                    //compare where the map says the robot should be and where the new update says the robot should be
                    RobotPos mapPos = this.getRobotPositionAtTime(update.timestamp);
                    if (mapPos == null) {
                        // if there was an error getting the position from the map, ignore the update
                        System.out.println("COULD NOT ADD CAMERA UDPATE TO ROBOT MAP");
                        return;
                    }

                    double delta = Point.getDistance(mapPos.position, new Point(update.getX(), update.getY()));
                    if (delta < Constants.kCameraToMapToleranceLvl1) {
                        this.rebaseMap(update); //if we are within lvl1 tolerances, completely rebase the map
                    } else if (delta < Constants.kCameraToMapToleranceLvl2) {
                        //if we are within lvl2 tolerances, partially rebase the map with percents of each update from the constants file
                        double newX = (update.getX() * Constants.kCameraToMapPercentLvl2) + (mapPos.getX() * (1-Constants.kCameraToMapPercentLvl2));
                        double newY = (update.getY() * Constants.kCameraToMapPercentLvl2) + (mapPos.getY() * (1-Constants.kCameraToMapPercentLvl2));
                        RobotPosUpdate newBase = new RobotPosUpdate(newX,newY, mapPos.heading, update.timestamp,UpdateType.BASE);
                        this.rebaseMap(newBase); 
                    } else if (delta < Constants.kCameraToMapToleranceLvl3) {
                        //if we are within lvl3 tolerances, partially rebase the map with percents of each update from the constants file
                        double newX = (update.getX() * Constants.kCameraToMapPercentLvl3) + (mapPos.getX() * (1-Constants.kCameraToMapPercentLvl3));
                        double newY = (update.getY() * Constants.kCameraToMapPercentLvl3) + (mapPos.getY() * (1-Constants.kCameraToMapPercentLvl3));
                        RobotPosUpdate newBase = new RobotPosUpdate(newX,newY, mapPos.heading, update.timestamp,UpdateType.BASE);
                        this.rebaseMap(newBase); 
                    }
                    return;
                } else {
                    // if the element's timestamp is older than the update's add it to the array at the old element's index
                    map.add(i, update);
                }
                
                break; // stop looking for older timestamps
            }
            // otherwise keep looking through the array for an older update
        }

        if (!foundHome && !update.isAbsolute()) {
            // there are no elements in the list older than the update, so make this one the first element
            map.add(0, update);
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
                return new int[] {mid};
            } else if (map.get(mid).getTimestamp() < timestamp) {
                last = mid - 1;
            } else {
                first = mid + 1;
            }
        }
        if (last >= 0 && first < map.size()) {
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
     * Gets an estimate of the robot position at the given time
     * 
     * @param time the time at which the position of the robot is desired
     * @return the robot's position at the given time
     */
    public RobotPos getRobotPositionAtTime(double time) {
        RobotPosUpdate base = map.get(map.size() - 1);
        if (base.timestamp > time) {
            return null; // a position cannot be calculated for the given time, image too old
        }

        int[] indexArr = this.getMapIndexForTimestamp(time);
        if (indexArr.length == 0) {
            // the position cannot be calculated for the given time.
            return null;
        }
        if (indexArr.length == 1) {
            // there is a whole number index at the given time, calculate position at that time
            int index = indexArr[0];

            double xTot = base.getX();
            double yTot = base.getY();
            double heading = map.get(index).heading;
            
            //add all elements after the one at the requested time
            for (int i = index; i < map.size() - 1; i++) {
                RobotPosUpdate rel = map.get(i);
                xTot += rel.getDx();
                yTot += rel.getDy();
            }
            return new RobotPos(new Point(xTot, yTot), heading, 0, 0);
        }
        if (indexArr.length == 2) {
            // there is no whole number index for the requested time, create a partial update representing
            // the movement in between the last update and that time
            RobotPosUpdate partialUpdate = RobotPosUpdate.createUpdateBetweenPrevUpdateAndTime(map.get(indexArr[0]), map.get(indexArr[1]), time);

            int prevUpdate = indexArr[0];

            double xTot = base.getX();
            double yTot = base.getY();

            for (int i = prevUpdate; i < map.size() - 1; i++) {
                RobotPosUpdate rel = map.get(i);
                xTot += rel.getDx();
                yTot += rel.getDy();
            }
            
            xTot += partialUpdate.getDx();
            yTot += partialUpdate.getDy();

            return new RobotPos(new Point(xTot, yTot), partialUpdate.getHeading(), 0, 0);
        }
        return null;
    }

    /**
     * Gets an estimate of the robot's displacement since the current time
     * 
     * @param time the time at which the position of the robot is desired
     * @return the robot's position at the given time
     */
    public RobotPos getRobotMovementSinceTime(double time) {
        if (this.map.size() < 2) {
            return null; // list is too small to extrapolate from
        }

        RobotPosUpdate firstRelative = map.get(map.size() - 2);
        if (firstRelative.timestamp > time) {
            return null; // movement cannot be calculated for the given time, image too old
        }

        int[] indexArr = this.getMapIndexForTimestamp(time);
        if (indexArr.length == 0) {
            // movement cannot be calculated for the given time.
            return null;
        }
        if (indexArr.length == 1) {
            // there is a whole number index at the given time, calculate movement position since that time
            int index = indexArr[0];

            double xTot = 0;
            double yTot = 0;
            double heading = map.get(0).heading; //heading is the heading of the newest update in the map
            
            //add all elements after the one at the requested time
            for (int i = index; i >= 0; i--) {
                RobotPosUpdate rel = map.get(i);
                xTot += rel.getDx();
                yTot += rel.getDy();
            }
            return new RobotPos(new Point(xTot, yTot), heading, 0, 0);
        }
        if (indexArr.length == 2) {
            // there is no whole number index for the requested time, create a partial update representing
            // the movement in between the last update and that time
            RobotPosUpdate partialUpdate = RobotPosUpdate.createUpdateBetweenTimeAndNextUpdate(map.get(indexArr[0]), map.get(indexArr[1]), time);

            int nextUpdate = indexArr[1];

            double xTot = partialUpdate.getDx();
            double yTot = partialUpdate.getDy();

            for (int i = nextUpdate-1; i >= 0; i--) {
                RobotPosUpdate rel = map.get(i);
                xTot += rel.getDx();
                yTot += rel.getDy();
            }
            return new RobotPos(new Point(xTot, yTot), partialUpdate.getHeading(), 0, 0);
        }
        return null;
    }

    /**
     * Rebases the robotPosMap with a new base
     * 
     * @param update a absolute RobotPosUpdate that should be the new base of the map
     */
    private void rebaseMap(RobotPosUpdate update) {
        // the base should be the last element in the list
        RobotPosUpdate oldBase = map.get(map.size() - 1);
        // remove the old base from the list
        map.remove(map.size()-1);

        //get the index(es) that the update should replace
        int[] indexArr = this.getMapIndexForTimestamp(update.getTimestamp());
        if (indexArr.length == 0) {
            // the map cannot be rebased, revert the old base and stop
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
            RobotPosUpdate partialUpdate = RobotPosUpdate.createUpdateBetweenTimeAndNextUpdate(map.get(indexArr[0]), map.get(indexArr[1]), update.getTimestamp());

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

    /**
     * Get the field relative position of the robot based on the current pos map
     * 
     * @return a RobotPos representing the current pos of the robot
     */
    public RobotPos getLastestFieldRelativePosition() { //TODO account for movement since last update?
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
        double latestHeading = MathHelper.angleToNegPiToPi(map.get(0).getHeading());
        return new RobotPos(new Point(xTot, yTot), latestHeading, 0, 0);
    }

    /**
     * print the given map to the terminal
     */
    public void printMap() {
        for (RobotPosUpdate update : map) {
            System.out.println(update);
        }
    }

    /**
     * says if the path is should integrate or ignore camera updates
     * @param allowed true if camera updates should be integrated, false if they should be ignored
     */
    public void setAllowedToIntegrateCameraUpdates(boolean allowed) {
        this.allowsCameraUpdates = allowed;
    }

    /**
     * Clears the update map and sets the current position of the map to the given position
     * 
     * @param update the new position to set the map to
     */
    public void clearAndSetPos(RobotPos newPos) {
        // clear the map
        map = new ArrayList<RobotPosUpdate>();
        map.add(new RobotPosUpdate(newPos.getX(), newPos.getY(), newPos.getHeading(), 0, UpdateType.BASE));
    }

}