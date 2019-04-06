package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.RobotPos;
import frc.lib.TargetUpdate;
import frc.robot.Robot;
import frc.robot.VectorManager;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Segment;


public class DriveToTarget extends CommandGroup {
    
    public DriveToTarget() {
        TargetUpdate targetUpdate = null;   
        
        

        if(Robot.reflectiveTapeCamera.getUpdate() == null){
            end();
            return;
        }

        targetUpdate = Robot.reflectiveTapeCamera.getUpdate();
        
        VectorManager vision = new VectorManager();
        Path pathA = new Path();
        Path pathB = new Path();
        Segment lineA;
        Segment lineB;
        
        Robot.gyro.zeroYaw();
        Robot.estimator.zero();

        vision.setTargetUpdate(targetUpdate);
        vision.setRobotPos(Robot.estimator.getPos());
        
        double angleA = vision.getAngleA();
        addSequential(new TurnToAngle(angleA));

        vision.setRobotPos(Robot.estimator.getPos());
        lineA = vision.calcVisionLineSegmentA();
        pathA.addSegment(lineA);
        addSequential(new FollowPath(pathA));

        addSequential(new TurnToAngle(vision.getAngleB(angleA)));

        vision.setRobotPos(Robot.estimator.getPos());
        lineB = vision.calcVisionLineSegmentB();
        pathB.addSegment(lineB);
        addSequential(new FollowPath(pathB));

    }
}