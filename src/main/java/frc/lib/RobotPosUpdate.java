package frc.lib;

/**
 * a class responcible for storing a positional update from sensors
 */
public class RobotPosUpdate {

    double x = 0;
    double y = 0;

    double dx = 0;
    double dy = 0;

    double heading = 0;

    double timestamp;
    
    public static enum UpdateType {
        WHEEL,  // an update from wheel odometry
        CAMERA, // an update from the camera
        BASE,   // information that was combined into a single datapoint to save space 
                // in the map and is now the starting point of pos estimation
       
        EXTRAPOLATION // an update that calculates how much the robot has moved
                      // since the last encoder update using forward kinematics //TODO IMPLEMENT
    };

    UpdateType type;

    public RobotPosUpdate(double x, double y, double heading, double timestamp, UpdateType type) {
        this.type = type;
        this.timestamp = timestamp;
        this.heading = heading; // in radians

        if (type == UpdateType.WHEEL) {
            this.dx = x;
            this.dy = y;
        } else {
            this.x = x;
            this.y = y;
        }
    }
    
    /**
     * Gets the timestamp in seconds
     * 
     * @return timestamp
     */
    public double getTimestamp() {
        return this.timestamp;
    }

    /**
     * tells whether the update has field relative x and y coords or has dx and dy coords relative to the last update
     * 
     * @return whether or not the update is absolute
     */
    public boolean isAbsolute() {
        return this.type != UpdateType.WHEEL;
    }

    /**
     * Gets the change in x from non-absolute updates
     * 
     * @return the dx of a (non-absolute) update
     */
    public double getDx() {
        if (!this.isAbsolute()) {
            return dx;
        } else {
            System.out.println("ERROR: ATTEMPTED TO GET DX FROM AN ABSOLUTE UPDATE POINT");
            return 5;
        }
    }

    /**
     * Gets the change in y from non-absolute updates
     * 
     * @return the dy of a (non-absolute) update
     */
    public double getDy() {
        if (!this.isAbsolute()) {
            return dy;
        } else {
            System.out.println("ERROR: ATTEMPTED TO GET DY FROM AN ABSOLUTE UPDATE POINT");
            return 0;
        }
    }

    /**
     * Gets the field relative x-coord of an absolute update
     * 
     * @return field relative x-coord of the update
     */
    public double getX() {
        if (this.isAbsolute()) {
            return this.x;
        } else {
            System.out.println("ERROR: ATTEMPTED TO GET X FROM A NON-ABSOLUTE UPDATE");
            return 0;
        }
    }

    /**
     * Gets the field relative y-coord of an absolute update
     * 
     * @return field relative y-coord of the update
     */
    public double getY() {
        if (this.isAbsolute()) {
            return this.y;
        } else {
            System.out.println("ERROR: ATTEMPTED TO GET Y FROM A NON-ABSOLUTE UPDATE");
            return 0;
        }
    }
    
    /**
     * gets the heading (in radians) of the robot at the update time
     * @return the heading of the robot at the update time
     */
    public double getHeading() {
        return this.heading;
    }

    /**
     * adds data from a non-absolute RobotPosUpdate to a base, effectively merging the updates into a new base
     * 
     * @param newUpdate a non-absolute RobotPosUpdate
     * @return a new base-type RobotPosUpdate containing the absolute x and y from the current object combined with the
     *         dx and dy of a new update
     */
    public RobotPosUpdate consolidateBaseWithUpdate(RobotPosUpdate newUpdate) {
        if (this.type == UpdateType.BASE) {
            if (!newUpdate.isAbsolute()) {
                double newX = this.getX() + newUpdate.getDx();
                double newY = this.getY() + newUpdate.getDy();
                return new RobotPosUpdate(newX, newY, heading, newUpdate.getTimestamp(), UpdateType.BASE);
            } else {
                System.out.println("ERROR: ATTEMPTED TO CONSOLIDATE A BASE WITH A NON-ABSOLUTE ROBOTPOS UPDATE");
                return this;
            }
        } else {
            System.out.println("ERROR: ATTEMPTED TO CONSOLIDATE WITH A NON-BASE TYPE ROBOTPOS UPDATE");
            return null;
        }
    }

    /**
     * Takes an absolute robotPosUpdate and makes it a base
     */
    public RobotPosUpdate makeBase() {
        if (this.isAbsolute()) {
            return new RobotPosUpdate(this.getX(), this.getY(), this.getHeading(), this.timestamp, UpdateType.BASE);
        } else {
            System.out.println("ERROR: ATTEMPTED TO TURN A NON-ABSOLUTE ROBOTPOS UPDATE INTO A BASE");
            return null;
        }
    }
    
    /**
     * Creates a partial update between the prevUpdate and the timestamp by linearly interpolating x and y over the time
     * Update is between timestamp and nextUpdate
     * 
     * @param prevUpdate a relative update before the timestamp
     * @param nextUpdate a relative update after the timestamp
     * @param timestamp the time that the relative update is requested for
     * @return a RobotPosUpdate representing the movement between prevUpdate and the timestamp
     */
    public static RobotPosUpdate createUpdateBetweenTimeAndNextUpdate(RobotPosUpdate prevUpdate, RobotPosUpdate nextUpdate, double timestamp) {
        double dtTotal = nextUpdate.getTimestamp() - prevUpdate.getTimestamp(); // the amount of time the next update encompases
        double dtDesired = nextUpdate.getTimestamp() - timestamp; // the amount of time we want the update to encompass

        double percentDesired = dtDesired / dtTotal;

        double dx = nextUpdate.getDx() * percentDesired;
        double dy = nextUpdate.getDy() * percentDesired;

        return new RobotPosUpdate(dx, dy, nextUpdate.getHeading(), nextUpdate.getTimestamp(), UpdateType.WHEEL);
    }

     /**
     * Creates a partial update between the prevUpdate and the timestamp by linearly interpolating x and y over the time
     * Update is between prevUpdate and timestamp
     * 
     * @param prevUpdate a relative update before the timestamp
     * @param nextUpdate a relative update after the timestamp
     * @param timestamp the time that the relative update is requested for
     * @return a RobotPosUpdate representing the movement between prevUpdate and the timestamp
     */
    public static RobotPosUpdate createUpdateBetweenPrevUpdateAndTime(RobotPosUpdate prevUpdate, RobotPosUpdate nextUpdate, double timestamp) {
        double dtTotal = nextUpdate.getTimestamp() - prevUpdate.getTimestamp(); // the amount of time the next update encompases
        double dtDesired = nextUpdate.getTimestamp() - timestamp; // the amount of time we want the update to encompass

        double percentDesired = 1 - (dtDesired / dtTotal);

        double dx = nextUpdate.getDx() * percentDesired;
        double dy = nextUpdate.getDy() * percentDesired;
        double dh = (nextUpdate.getHeading() - prevUpdate.getHeading()) * percentDesired;

        double heading = MathHelper.angleToNegPiToPi(prevUpdate.getHeading() + dh);

        return new RobotPosUpdate(dx, dy, heading, timestamp, UpdateType.WHEEL);
    }

    @Override
    public String toString() {
        if (this.type == UpdateType.WHEEL) {
            return String.format("Type: WHEEL; dx: %s, dy: %s; timestamp: %s", this.dx, this.dy, this.timestamp);
        } else if (this.type == UpdateType.CAMERA) {
            return String.format("Type: CAMERA; x: %s, y: %s; timestamp: %s", this.x, this.y, this.timestamp);
        } else if (this.type == UpdateType.BASE) {
            return String.format("Type: BASE; dx: %s, dy: %s; timestamp: %s", this.x, this.y, this.timestamp);
        } else {
            return "Update has invalid type!";
        }
    }
}