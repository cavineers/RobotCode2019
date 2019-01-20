package frc.lib.dubinPath;

import java.util.ArrayList;

import frc.lib.pathPursuit.Segment;

public class DubinsTrajectory {
    public TrajectoryType type;
    public ArrayList<Control> controls;
    public double length;
    public ArrayList<Segment> segmentList;

    DubinsTrajectory() {
        this.type = TrajectoryType.RSR;
        this.controls = new ArrayList<Control>(3);
        segmentList = new ArrayList<Segment>();
        this.length = 1e9;
    }

    public void addSegment(Segment seg) {
        segmentList.add(seg);
    }

    public ArrayList<Segment> getSegmentList() {
        return segmentList;
    }
}