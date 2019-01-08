package frc.robot.commands;

import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.RobotCmd;
import frc.lib.RobotPos;
import frc.lib.pathPursuit.Segment;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class FollowPath extends Command {
	public Path path;

	public static enum PATH_TYPE {
		TEST_PATH, TEST_PATH_CURVE, TEST_PATH_CURVE_REVERSE, TEST_PATH_REVERSE
	}
	
	public FollowPath(PATH_TYPE pathType) {
		this.path = this.getPathFromType(pathType);
		requires(Robot.drivetrain);
	}
	
	@Override
	protected void initialize() {
		Robot.drivetrain.setLeftVel(0);
		Robot.drivetrain.setRightVel(0);
	}
	
	public Path getPathFromType(PATH_TYPE pathType) {
		switch (pathType) {
		case TEST_PATH: {
			path = new Path();
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 60, 0);
			path.addSegment(seg1);
			return path;
		} case TEST_PATH_REVERSE: {
			path = new Path(false, true, Constants.kMaxAccelSpeedUp);
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(-60, 0), 30);
			path.addSegment(seg1);
			Segment seg2 = new ArcSegment(new Point(-60, 0), new Point(-90, -30), new Point(-60, -30), 30,0);
			path.addSegment(seg2);
			return path;
		} case TEST_PATH_CURVE: {
			path = new Path();
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 30, 30);
			path.addSegment(seg1);
			Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, -30), new Point(60, -30), 30);
			path.addSegment(seg2);
			Segment seg3 = new ArcSegment(new Point(90, -30), new Point(120, -60), new Point(120, -30), 30);
			path.addSegment(seg3);
			Segment seg4 = new LineSegment(new Point(120, -60), new Point(170, -60), 30, 0);
			path.addSegment(seg4);
			return path;
		} case TEST_PATH_CURVE_REVERSE: {
			path = new Path(false, true, Constants.kMaxAccelSpeedUp);
			Segment seg1 = new LineSegment(new Point(170, -60), new Point(120, -60), 30);
			path.addSegment(seg1);
			Segment seg2 = new ArcSegment(new Point(120, -60), new Point(90, -30), new Point(120, -30), 30);
			path.addSegment(seg2);
			Segment seg3 = new ArcSegment(new Point(90, -30), new Point(60, 0), new Point(60, -30), 30);
			path.addSegment(seg3);
			Segment seg4 = new LineSegment(new Point(60, 0), new Point(0, 0), 30, 0);
			path.addSegment(seg4);
			return path;
		} 
		default: {
			return null;
		}
		}
	}
	
	@Override
	public void execute() {
		RobotPos latestPos = new RobotPos(Robot.estimator.getPosition(),
				Robot.estimator.getHeading(), Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel()); //right way

		RobotCmd cmd = this.path.update(latestPos);
		
		//debug print: lTarget, rTarget, lActualVel, rActualVel, RobotPosition
//		System.out.println(Robot.drivetrain.leftMotor1.getClosedLoopTarget(0) +  "," + Robot.drivetrain.rightMotor1.getClosedLoopTarget(0) + "," + Robot.drivetrain.leftMotor1.getSelectedSensorVelocity(0) + "," + Robot.drivetrain.rightMotor1.getSelectedSensorVelocity(0) + "," + Robot.state.getPosition());
		System.out.println(Robot.estimator.getX() + "," + Robot.estimator.getY());
		
		Robot.drivetrain.setLeftVel(cmd.getLeftVel());
		Robot.drivetrain.setRightVel(cmd.getRightVel());
	}
	
	@Override
	protected boolean isFinished() {
		return this.path.isFinished();
	}
	
	@Override
	protected void end() {
		Robot.drivetrain.setLeftVel(0);
		Robot.drivetrain.setRightVel(0);
		if (this.isFinished()) {
			System.out.println("FINISHED PATH");
			System.out.println(Robot.estimator.getPosition());
		}
	}

}
