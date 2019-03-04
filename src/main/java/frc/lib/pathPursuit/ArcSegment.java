package frc.lib.pathPursuit;

import frc.lib.MathHelper;
import frc.lib.Vector2D;
import frc.robot.Constants;

public class ArcSegment implements Segment {
    
    private Point startPoint;
    private Point endPoint;
    private Point centerPoint;
    
    private Point deltaStart;
    
    double radius;
    
    double maxVel;
    double endVel;
    
    boolean isAccelToEndpoint = false;

    TURN smallSide;
    public enum TURN {
        RIGHT,
        LEFT,
        SMALL_SIDE,
        LARGE_SIDE
    }

    public TURN turnType;
    
    public ArcSegment(Point startPoint, Point endPoint, Point centerPoint, double maxVel, double endVel, TURN turnType) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.centerPoint = centerPoint;
        
        this.radius = Point.getDistance(this.startPoint, this.centerPoint);
        
        this.deltaStart = Point.getDelta(centerPoint, startPoint);
        
        this.maxVel = maxVel;
        this.endVel = endVel;
        
        isAccelToEndpoint = false;

        // the vector from the center of the circle to the start
        Vector2D centerToStart = new Vector2D(this.centerPoint.getX() - this.startPoint.getX(), this.centerPoint.getY() - this.startPoint.getY());
        Vector2D centerToEnd   = new Vector2D(this.centerPoint.getX() - this.endPoint.getX(), this.centerPoint.getY() - this.endPoint.getY());
        
        double crossProduct = Vector2D.getCrossProduct(centerToStart, centerToEnd);
        
        // figure out if the small side of the circle is a left or right hand turn
        if (crossProduct < 0) {
            this.smallSide = TURN.RIGHT;
        } else {
            this.smallSide = TURN.LEFT;
        }

        this.turnType = turnType;

        if (turnType == TURN.SMALL_SIDE) {
            this.turnType = this.smallSide;
        } else if (turnType == TURN.LARGE_SIDE) {
            //go for the opposite turn of the small side if the large side is desired
            if (this.smallSide == TURN.LEFT) {
                this.turnType = TURN.RIGHT;
            } else {
                this.turnType = TURN.LEFT;
            }
        }

    }

    public ArcSegment(Point startPoint, Point endPoint, Point centerPoint, double maxVel, double endVel) {
        this(startPoint, endPoint, centerPoint, maxVel, endVel, TURN.SMALL_SIDE);
    }
    
    public ArcSegment(Point startPoint, Point endPoint, Point centerPoint, double maxVel) {
        this(startPoint, endPoint, centerPoint, maxVel, maxVel);
    }

    public ArcSegment(Point startPoint, Point endPoint, Point centerPoint) {
        this(startPoint, endPoint, centerPoint, Constants.kMaxTargetSpeed);
    }
    
    /**
     * Gets the closest point on the arc segment to the given point.  Approach was modified from
     * Team 254's on github:
     * https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/lib/util/control/PathSegment.java
     * 
     * @param lookaheadPoint: the robot's position + lookahead
     */
    @Override
    public Point getClosestPointOnSegment(Point lookaheadPoint) {
        // Map the lookahead point to the closest point on the circle
         Point deltaPose = Point.getDelta(centerPoint, lookaheadPoint);
        double scale = deltaStart.getHypot() / deltaPose.getHypot();
        deltaPose = new Point(deltaPose.getX() * scale, deltaPose.getY() * scale);
        Point arcPoint = Point.addPoints(centerPoint, deltaPose);
        
        // Get the angle of the arcs between the circle's point and the endpoints of the arc
        double startAngle = MathHelper.getAngleForArc(startPoint, arcPoint, centerPoint);
        double endAngle = MathHelper.getAngleForArc(arcPoint, endPoint, centerPoint);
        double arcAngle = MathHelper.getAngleForArc(startPoint, endPoint, centerPoint);

        //TODO: account for arcs that are exactly Pi radians

        // if the arcs between the endpoints equal the total length of the arc, we are on the smaller arc, otherwise we are on the larger arc
        if ((MathHelper.areApproxEqual(startAngle + endAngle, arcAngle) && smallSide == this.turnType) || !(MathHelper.areApproxEqual(startAngle + endAngle, arcAngle) && smallSide != this.turnType)) {
            // We are on the part of the arc we want, just return the point
            return arcPoint;
        } else { 
            // We are not on the arc so return the closest endpoint
            if (Point.getDistance(arcPoint, startPoint) < Point.getDistance(arcPoint, endPoint)) {
                return startPoint;
            } 
            return endPoint;
        }
    }

    @Override
    public Point getStartPoint() {
        return this.startPoint;
    }

    @Override
    public Point getEndPoint() {
        return this.endPoint;
    }
    
    public Point getCenterPoint() {
        return this.centerPoint;
    }

    @Override
    public double getDistanceToEndpoint(Point lookaheadPos) {
        //get the arc angle of the amount of distance left to drive
        double remainingArcRadians = MathHelper.getAngleForArc(lookaheadPos, this.endPoint, this.centerPoint);
        
        if (this.smallSide != this.turnType) {
            //we are on the larger part of a circle

            //create vectors from the center to the start point, lookahead point, and end point
            Vector2D centerToEnd = Point.getDelta(this.endPoint, this.centerPoint).getVector();
            Vector2D centerToLPos = Point.getDelta(this.startPoint, lookaheadPos).getVector();

            double crossProduct = Vector2D.getCrossProduct(centerToLPos, centerToEnd);

            TURN travelDir;

            // figure out if the small side of the circle is a left or right hand turn
            if (crossProduct < 0) {
                travelDir = TURN.RIGHT;
            } else {
                travelDir = TURN.LEFT;
            }

            if (this.turnType == travelDir) {
                remainingArcRadians = (Math.PI * 2) - remainingArcRadians;
            }
        }
        
        //get total arc distance with s = theta * r
        return remainingArcRadians * this.radius;
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
        
        if (this.getDistanceToEndpoint(this.getClosestPointOnSegment(robotPosition)) < lookahead) {
            return this.getEndPoint();
        }
        
        //calculated needed arc angle with s = theta * r
        double theta = lookahead / this.radius;
        Point closestPoint = this.getClosestPointOnSegment(robotPosition);
        
        //Rotate the closest point by positive and negative theta
        double x1 = Math.cos(theta) * (closestPoint.getX() - this.centerPoint.getX()) - Math.sin(theta) * (closestPoint.getY() - this.centerPoint.getY()) + this.centerPoint.getX();
        double y1 = Math.sin(theta) * (closestPoint.getX() - this.centerPoint.getX()) + Math.cos(theta) * (closestPoint.getY() - this.centerPoint.getY()) + this.centerPoint.getY();
        Point p1 = new Point(x1, y1);
        
        double x2 = Math.cos(-theta) * (closestPoint.getX() - this.centerPoint.getX()) - Math.sin(-theta) * (closestPoint.getY() - this.centerPoint.getY()) + this.centerPoint.getX();
        double y2 = Math.sin(-theta) * (closestPoint.getX() - this.centerPoint.getX()) + Math.cos(-theta) * (closestPoint.getY() - this.centerPoint.getY()) + this.centerPoint.getY();
        
        Point p2 = new Point(x2, y2);
        
        //create vectors from the center to the start point, lookahead point, and end point
        Vector2D centerToEnd = Point.getDelta(robotPosition, this.centerPoint).getVector();
        Vector2D centerToLPos = Point.getDelta(p1, this.centerPoint).getVector();
        // Vector centerToRPos = Point.getDelta(robotPosition, this.cen)

        double crossProduct = Vector2D.getCrossProduct(centerToLPos, centerToEnd);

        TURN travelDir;

        // figure out if the small side of the circle is a left or right hand turn
        if (crossProduct > 0) {
            travelDir = TURN.RIGHT;
        } else {
            travelDir = TURN.LEFT;
        }

        if (travelDir == this.turnType) {
            return p1;
        } else {
            return p2;
        }
    }

}
