package frc.lib.pathPursuit;

public interface Segment {
    
    public Point getClosestPointOnSegment(Point lookaheadPoint);
    
    public Point getStartPoint();
    
    public Point getEndPoint();
    
    public double getDistanceToEndpoint(Point lookaheadPos);
    
    public Point getLookaheadPoint(Point robotPosition, double lookahead);
    
    public double getMaxVelocity();
    
    public double getEndVelocity();
    
    public void setIsAcceleratingToEndpoint(boolean isAccel);
    
    public boolean isAcceleratingToEndpoint();
    
}
