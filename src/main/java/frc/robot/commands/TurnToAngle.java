package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.lib.MathHelper;
import frc.lib.VelocityTrapezoid;

public class TurnToAngle extends Command {
  double setpoint = 0;
  VelocityTrapezoid velTrapezoid = new VelocityTrapezoid(Constants.kMaxAccelSpeedUp, Constants.kMaxTurnToAngleSpeed, Constants.kDefaultDt);

  public TurnToAngle(double angleDeg) {
    requires(Robot.drivetrain);
    this.setpoint = Math.toRadians(angleDeg);
  }

  @Override
  protected void initialize() {
    Robot.drivetrain.setLeftVel(0);
    Robot.drivetrain.setRightVel(0);
  }

  @Override
  protected void execute() {

    // calculate error between current angle and setpoint
    double error = this.setpoint - Robot.getAngleRad();
    error = MathHelper.angleToNegPiToPi(error);

    // calculate the distance that wheels must travel in order to reach that setpoint
    double dist = calculateWheelDistanceNeeded(error);

    // find the average speed of the wheels
    double avgWheelSpeed = (Math.abs(Robot.drivetrain.getLeftVel()) + Math.abs(Robot.drivetrain.getRightVel())) / 2;

    // figure out how fast the robot should be turning given acceleration and speed constraints
    double turningSpeed = velTrapezoid.update(avgWheelSpeed, dist);

    // figure out the direction that the robot should be turning
    if (error > 0) {
      //robot is turning right
      Robot.drivetrain.setLeftVel(turningSpeed);
      Robot.drivetrain.setRightVel(-turningSpeed);
    } else if (error < 0){
      //robot is turning left
      Robot.drivetrain.setLeftVel(-turningSpeed);
      Robot.drivetrain.setRightVel(turningSpeed);
    } else {
      // there is no error so don't rotate
      Robot.drivetrain.setLeftVel(0);
      Robot.drivetrain.setRightVel(0);
    }

  }

  /**
   * uses s = theta * r to calculate how much each side of the robot must turn
   * to eliminate the error
   * 
   * @param angleError angle error in  radians
   * @return feet required for the wheels to turn
   */
  private double calculateWheelDistanceNeeded(double angleErrorRad) {
    return angleErrorRad * (Constants.kWheelBase / 2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double error = this.setpoint - Robot.getAngleRad();
    return error <= Constants.kAngleTolerance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.setLeftVel(0);
    Robot.drivetrain.setRightVel(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drivetrain.setLeftVel(0);
    Robot.drivetrain.setRightVel(0);
  }
}
