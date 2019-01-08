package frc.lib.tests;

import frc.robot.Constants;
import frc.lib.MathHelper;
import frc.lib.VelocityTrapezoid;
import frc.lib.RobotPos;

public class TurnToAngleTest {

	public static void main(String[] args) {
		RobotPos currentPos = new RobotPos(0,0, 0, 0,0);
		double setpoint = Math.toRadians(180);
		VelocityTrapezoid velTrapezoid = new VelocityTrapezoid(Constants.kMaxAccelSpeedUp, Constants.kMaxTurnToAngleSpeed, Constants.kDefaultDt);
		int counter = 0;

		while (true) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			// calculate error between current angle and setpoint
			double error = setpoint - currentPos.getHeading();
			error = MathHelper.angleToNegPiToPi(error);
			if (Math.abs(error) < Math.toRadians(Constants.kAngleTolerance)) {
				break;
			}

			// calculate the distance that wheels must travel in order to reach that setpoint
			double dist = calculateWheelDistanceNeeded(error);

			// find the average speed of the wheels
			double avgWheelSpeed = (Math.abs(currentPos.getLeftVel()) + Math.abs(currentPos.getRightVel())) / 2;
		
			// figure out how fast the robot should be turning given acceleration and speed constraints
			double turningSpeed = velTrapezoid.update(avgWheelSpeed, dist);

			double lVel;
			double rVel;
			// figure out the direction that the robot should be turning
			if (error > 0) {
				//robot is turning right (?)
				lVel = turningSpeed;
				rVel = -turningSpeed;
			} else if (error < 0){
				//robot is turning left (?)
				lVel = -turningSpeed;
				rVel = turningSpeed;
			} else {
				// there is no error so don't rotate
				lVel = 0;
				rVel = 0;
			}

			double heading = currentPos.heading + ((lVel - rVel) / Constants.kWheelBase) * Constants.kDefaultDt;
			
			double xPos = currentPos.position.getX() + (lVel + rVel)/2 * Constants.kDefaultDt * Math.cos(heading);
			double yPos = currentPos.position.getY() + (lVel  + rVel)/2 * Constants.kDefaultDt * Math.sin(heading);
			currentPos = new RobotPos(xPos, yPos, heading, rVel, lVel);
			System.out.println(currentPos);

			counter++;
		}
		System.out.println("Time Required: " + counter * Constants.kDefaultDt);
	}

	/**
	 * uses s = theta * r to calculate how much each side of the robot must turn
	 * to eliminate the error
	 * 
	 * @param angleError angle error in  radians
	 * @return feet required for the wheels to turn
	 */
	private static double calculateWheelDistanceNeeded(double angleErrorRad) {
	  return angleErrorRad * (Constants.kWheelBase / 2);
	}
}
