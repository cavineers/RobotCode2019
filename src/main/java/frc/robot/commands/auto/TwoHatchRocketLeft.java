package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoPathHelper;
import frc.robot.Robot;
import frc.robot.commands.FollowPath;
import frc.robot.commands.grabber.HomeGrabber;

public class TwoHatchRocketLeft extends CommandGroup{
    public TwoHatchRocketLeft() {
        addSequential(new Command() {
            @Override
            protected void initialize() {
                System.out.println("starting...");
                Robot.gyro.zeroYaw();
                Robot.estimator.setPos(AutoPathHelper.getPath(AutoPathHelper.PATH_TYPE.LEFT_ROCKET_1).getCurrentSegment().getStartPoint());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
            
        });
        //initial homing routine
        addSequential(new HomeGrabber());
        //drive from the hab to the far side of the left rocket
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.LEFT_ROCKET_1));
        //place the hatch

        //back up and go to a turnaround point
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.LEFT_ROCKET_2));
        //finish turning around and drive to the exchange
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.LEFT_ROCKET_3));
        //pick up the new hatch

        //drive backwards to a turnaround point
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.LEFT_ROCKET_4));
        //drive to the near side of the left rocket
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.LEFT_ROCKET_5));
        //place the second hatch
    }
}