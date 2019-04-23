package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.RobotPos;
import frc.lib.TargetUpdate;
import frc.robot.Robot;
import frc.robot.VectorManager;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Segment;

public class DriveToTarget extends Command {

    int step = 0;
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

	public DriveToTarget() {
		requires(Robot.drivetrain);
	}

	// Called just before this Command runs the first time
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

        step = 1;
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		switch (step) {
		case 1:
			if (turningCmdA != null && turningCmdA.isCompleted()) {
				vision.setRobotPos(Robot.estimator.getPos());
                lineA = vision.calcVisionLineSegmentA();
                pathA.addSegment(lineA);

                followPathA = new FollowPath(pathA);
                followPathA.start();
                angleB = vision.getAngleB(angleA);
				step = 2;
			}
			break;

		case 2:
			if (followPathA != null && followPathA.isCompleted()) {
				turningCmdB = new TurnToAngle(Math.toDegrees(angleB));
                turningCmdB.start();
				step = 3;
			}
            break;
        
        case 3:
            if(turningCmdB != null && turningCmdB.isCompleted()){
                vision.setRobotPos(Robot.estimator.getPos());
                lineB = vision.calcVisionLineSegmentB();
                pathB.addSegment(lineB);
                followPathB = new FollowPath(pathB);
                followPathB.start();
                step = 4;
            }
        

		}

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return step == 4;

	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

}
