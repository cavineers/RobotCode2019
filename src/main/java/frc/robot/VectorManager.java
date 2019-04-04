package frc.robot;
import frc.lib.RobotPos;
import frc.lib.TargetUpdate;
import frc.lib.Vector2D;
import frc.lib.Vector3D;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Point;
import frc.robot.Constants;

public class VectorManager {

    private TargetUpdate targetUpdate;
    private RobotPos robotFieldPos;
    private Vector3D vhtcs;
    private Vector3D vctts;
    private Vector3D vhtrs;
    private Vector2D v2trfs;
    private Point pointB;

    public void setTargetUpdate(TargetUpdate newTargetUpdate){
        this.targetUpdate = newTargetUpdate;
    }

    public TargetUpdate getTargetUpdate(){
        return this.targetUpdate;
    }

    //targetHeadingVect
    public Vector3D getVhtcs(){
        return getTargetUpdate().getVhtcs();
    }

    //targetVect
    public Vector3D getVctts(){
        return getTargetUpdate().getVctts();
    }

    //cameraVect
    public Vector3D getVtccs(){
        return getTargetUpdate().getVtccs();
    }

    //transform Target Heading Vector into robot coordinate system
    public Vector3D getVhtrs(){
        return getVhtcs().rotate(Constants.kMrscs);
    }

    //transform Camera Vector into robot coordinate system
    public Vector3D getVtrrs(){
        Vector3D vtcrs_rotated = getVtccs().rotate(Constants.kMrscs);
        return Vector3D.add(vtcrs_rotated, Constants.kVrscs);
    }

    //the target in a coordinate system alligned with the field, but centered at the robot
    public Vector3D getVtrfs(){
        return getVtrrs().rotateZAxis(robotFieldPos.getHeading());
    }

    public RobotPos getRobotPos(){
        return this.robotFieldPos;
    }

    public void setRobotPos(RobotPos newRobotFieldPos){
        this.robotFieldPos = newRobotFieldPos;
    }

    public Vector2D getV2trfs(){
        return new Vector2D((getVtrfs().getDx() - 8.0) + getRobotPos().getX(), getVtrfs().getDy() + getRobotPos().getY());
    }

    public double getTargetHeadingAngle(){
        return getRobotPos().getHeading() + Math.atan((getVhtrs().getDy()/getVhtrs().getDx()));       

    }

    //TODO work out which one is actually x and actually y
    public double calcAngle(Point startPoint, Point endPoint){
        double angle = 0;
        double aVal = Math.abs(startPoint.getX()- endPoint.getX());
        double bVal = Math.abs(startPoint.getY()- endPoint.getY());
        angle = Math.atan(aVal/bVal);
        return angle;
    }

    public Point getPointB(){
        return new Point(getVtrfs().getDx(), (getVtrfs().getDy()+Constants.kSpaceToTurn));  
    }

    public LineSegment calcVisionLineSegmentA(){
        Point robotPosPoint = new Point(getRobotPos().getX(), getRobotPos().getY());
        return new LineSegment(robotPosPoint, getPointB(), Constants.kMaxTargetSpeed, 0);
    }

    public LineSegment calcVisionLineSegmentB(){
        Point robotPosPoint = new Point(getRobotPos().getX(), getRobotPos().getY());
        Point targetPoint = new Point(getVtrfs().getDx(), getVtrfs().getDy());
        return new LineSegment(robotPosPoint, targetPoint, Constants.kMaxTargetSpeed, 0);
    }

    public double getAngleA(){
        Point robotPosPoint = new Point(getRobotPos().getX(), getRobotPos().getY());
        return calcAngle(robotPosPoint, getPointB());
    }

    public double getAngleB(double angleA){
        return (Math.PI - angleA);
    }

}