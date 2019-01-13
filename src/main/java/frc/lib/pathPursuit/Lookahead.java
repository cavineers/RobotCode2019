package frc.lib.pathPursuit;

/**
 * A utility class for interpolating lookahead distance based on current speed.
 * Taken from Team 254's 2017 github: 
 * https://github.com/Team254/FRC-2017-Public/blob/ab2b7a0c40a76d64e4be4f1c0471d8f2b3a4bcd4/src/com/team254/lib/util/control/Lookahead.java
 */
public class Lookahead {
    public final double min_distance;
    public final double max_distance;
    public final double min_speed;
    public final double max_speed;

    protected final double delta_distance;
    protected final double delta_speed;

    public Lookahead(double min_distance, double max_distance, double min_speed, double max_speed) {
        this.min_distance = min_distance;
        this.max_distance = max_distance;
        this.min_speed = min_speed;
        this.max_speed = max_speed;
        delta_distance = max_distance - min_distance;
        delta_speed = max_speed - min_speed;
    }

    public double getLookaheadForSpeed(double speed) {
        speed = Math.abs(speed);
        double lookahead = delta_distance * (speed - min_speed) / delta_speed + min_distance;
        return Double.isNaN(lookahead) ? min_distance : Math.max(min_distance, Math.min(max_distance, lookahead));
    }
}