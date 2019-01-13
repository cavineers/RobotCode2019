package frc.lib.pathPursuit.tests;

import frc.robot.Constants;
import frc.lib.RobotPos;
import frc.lib.pathPursuit.*;

public class ArcTest {
    
    /**
     * A class which helped test getClosestPointOnSegment with the help of Desmos Graphing Calculator
     * https://www.desmos.com/calculator/qdsqwrsrke
     */
    public static void main(String[] args) {
        RobotPos currentPos = new RobotPos(0,0, 0, 0,0);
        double currentVel = 20;
        
        ConnectionArc arc = new ConnectionArc(20, true);
        double lWheelVel = arc.getLeftVelocityTarget(currentVel);
        double rWheelVel = arc.getRightVelocityTarget(currentVel);
        
        while (true) {
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            RobotCmd cmd = new RobotCmd(lWheelVel, rWheelVel);
            
            double heading = currentPos.heading + ((cmd.getRightVel() - cmd.getLeftVel()) / Constants.kWheelBase) * Constants.kDefaultDt;
            
            double xPos = currentPos.position.getX() + (cmd.getLeftVel() + cmd.getRightVel())/2 * Constants.kDefaultDt * Math.cos(heading);
            double yPos = currentPos.position.getY() + (cmd.getLeftVel() + cmd.getRightVel())/2 * Constants.kDefaultDt * Math.sin(heading);
            currentPos = new RobotPos(xPos, yPos, heading, cmd.getRightVel(), cmd.getLeftVel());
            System.out.println(currentPos);
//			System.out.println((lWheelVel + rWheelVel) / 2);
        }
    }
    
    public static double ftToRad(double inches) {
        double circum = Constants.kWheelDiameter * Math.PI;
        return (inches / circum) * Math.PI * 2;
    }

}
