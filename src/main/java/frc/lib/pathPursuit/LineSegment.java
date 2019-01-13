package frc.lib.pathPursuit;

import frc.robot.Constants;

public class LineSegment implements Segment {
    
    private Point startPoint;
    private Point endPoint;
    
    double maxVel;
    double endVel;
    
    boolean isAccelToEndpoint = false;
    
    public LineSegment(Point startPoint, Point endPoint, double maxVel, double endVel) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        
        this.maxVel = maxVel;
        this.endVel = endVel;
    }
    
    public LineSegment(Point startPoint, Point endPoint, double maxVel) {
        this(startPoint, endPoint, maxVel, maxVel);
    }

    public LineSegment(Point startPoint, Point endPoint) {
        this(startPoint, endPoint, Constants.kMaxTargetSpeed);
    }
    
    /**
     * Gets the closest point on the line segment to the given point.  Approach was modified from
     * Joshua's on stack overflow:
     * https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
     * 
     * @param lookaheadPoint: the robot's position + lookahead
     */
    @Override
    public Point getClosestPointOnSegment(Point lookaheadPoint) {
        double A = lookaheadPoint.getX() - this.startPoint.getX();
        double B = lookaheadPoint.getY() - this.startPoint.getY();
        double C = this.endPoint.getX() - this.startPoint.getX();
        double D = this.endPoint.getY() - this.startPoint.getY();
        
        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        
        double param = -1;
        if (len_sq != 0) {
            param = dot / len_sq;
        }
        
        double xx, yy;
        
        if (param < 0) {
            xx = this.startPoint.getX();
            yy = this.startPoint.getY();
        }
        else if (param > 1) {
            xx = this.endPoint.getX();
            yy = this.endPoint.getY();
        }
        else {
            xx = this.startPoint.getX() + param * C;
            yy = this.startPoint.getY() + param * D;
        }
        
        return new Point(xx, yy);
    }

    @Override
    public Point getStartPoint() {
        return this.startPoint;
    }

    @Override
    public Point getEndPoint() {
        return this.endPoint;
    }
    
    @Override
    public double getDistanceToEndpoint(Point lookaheadPos) {
        return Point.getDistance(lookaheadPos, this.endPoint); 
    }

    @Override
    public double getMaxVelocity() {
        return this.maxVel;
    }

    @Override
    public double getEndVelocity() {
         return this.endVel;
    }
    
    @Override
    public void setIsAcceleratingToEndpoint(boolean isAccel) {
        this.isAccelToEndpoint = isAccel;
    }

    @Override
    public boolean isAcceleratingToEndpoint() {
        return this.isAccelToEndpoint;
    }
    
    @Override
    public Point getLookaheadPoint(Point robotPosition, double lookahead) {
        Point closestOnSegment = this.getClosestPointOnSegment(robotPosition);
        double distRemaining = this.getDistanceToEndpoint(closestOnSegment);
        if (distRemaining < lookahead) {
            return this.endPoint;
        }
        double segmentAngle = Point.getAngleNeeded(this.startPoint, this.endPoint);
        Point translationNeeded = new Point(Math.cos(segmentAngle) * lookahead, Math.sin(segmentAngle) * lookahead);
        
        Point p1 = Point.addPoints(closestOnSegment, translationNeeded);
        Point p2 = Point.subtractPoints(closestOnSegment, translationNeeded);
        
        if (Point.getDistance(p1, this.endPoint) < Point.getDistance(p2, endPoint)) {
            return p1;
        } else {
            return p2;
        }
    }

}
