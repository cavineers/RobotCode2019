package frc.robot;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.RobotPos;
import frc.lib.TargetUpdate;
import frc.lib.Vector2D;
import frc.lib.Vector3D;
import frc.lib.pathPursuit.Path;
import frc.robot.Constants;

public class VectorManager {

    Path path;
    TargetUpdate targetUpdate;
    RobotPos robotFieldPos;
    Vector3D vhtcs;
    Vector3D vctts;
    Vector3D vhtrs;
    Vector2D v2trfs;

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

    protected double calcAngle(double startX, double startY, double endX, double endY){
        double angle = 0;
        double xVal = Math.abs(startX- endX);
        double yVal = Math.abs(startY- endY);
        angle = Math.atan(yVal/xVal);
        return angle;
    }
}