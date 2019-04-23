package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.lib.TargetUpdate;
import frc.lib.pathPursuit.Path;
import frc.robot.Robot;
import frc.robot.VectorManager;
import frc.lib.pathPursuit.Segment;
import frc.robot.commands.FollowPath;

public class TravelToTarget extends Command {
    TargetUpdate targetUpdate = null; 
    Path pathA = new Path();
    Path pathB = new Path();
    Segment lineA;
    Segment lineB;
    VectorManager vision = new VectorManager();
    Command turningCmdA;
    Command turningCmdB;
    Command followPathA;
    Command followPathB;
    double angleA;
    double angleB;
    boolean step1Completed = false;
    boolean step2Completed = false;
    boolean step3Completed = false;


    public TravelToTarget() {
        requires(Robot.drivetrain);
        
    }

    @Override
    protected void initialize() {
        
        if(Robot.reflectiveTapeCamera.getUpdate() == null){
            end();
            return;
        }
        
        Robot.gyro.zeroYaw();
        Robot.estimator.zero();

        targetUpdate = Robot.reflectiveTapeCamera.getUpdate();

        vision.setTargetUpdate(targetUpdate);
        vision.setRobotPos(Robot.estimator.getPositionAtTime(targetUpdate.getTimestamp()));
        angleA = vision.getAngleA();

        turningCmdA = new TurnToAngle(Math.toDegrees(angleA));
        turningCmdA.start(); 
    }
  
    @Override
    protected void execute() {
        //GETS HERE CONSISTENTLY
        System.out.println(turningCmdA != null);
        System.out.println(turningCmdA.isCompleted());
        System.out.println(!step1Completed);
        if(turningCmdA != null && turningCmdA.isCompleted() && !step1Completed){
            System.out.println("Entered Step 1");
            vision.setRobotPos(Robot.estimator.getPos());
            lineA = vision.calcVisionLineSegmentA();
            pathA.addSegment(lineA);

            System.out.println(lineA);
            followPathA = new FollowPath(pathA);
            followPathA.start();
            System.out.println("DROVE LINE A");
            angleB = vision.getAngleB(angleA);
            step1Completed = true;
        }

        if(followPathA != null &&followPathA.isCompleted() && !step2Completed && step1Completed){
            turningCmdB = new TurnToAngle(Math.toDegrees(angleB));
            turningCmdB.start();
            step2Completed = true;
        }
                
        if(turningCmdB != null && turningCmdB.isCompleted() && !step3Completed && step2Completed && step1Completed){
            vision.setRobotPos(Robot.estimator.getPos());
            lineB = vision.calcVisionLineSegmentB();
            pathB.addSegment(lineB);
            followPathB = new FollowPath(pathB);
            followPathB.start();
            step3Completed = true;
            if(followPathB.isCompleted()){
                System.out.println("ENTERED AND ENDED COMMAND");
                end();
            }
        }
       
    }
  
    @Override
    protected boolean isFinished() {
        return true;
    }
  
    @Override
    protected void end() {
        System.out.println("CALLED END");
        isFinished();
    }
  
    @Override
    protected void interrupted() {
    }
}