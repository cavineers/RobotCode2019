package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoPathHelper;
import frc.robot.Robot;
import frc.robot.commands.FollowPath;
import frc.robot.commands.elevator.ElevatorToGround;
import frc.robot.commands.grabber.HomeGrabber;

public class TwoHatchRocketRight extends CommandGroup{
    public TwoHatchRocketRight() {
        //initial homing routine
        // addSequential(new HomeGrabber());
        // addSequential(new ElevatorToGround());
        addSequential(new Command() {
            @Override
            protected void initialize() {
                Robot.gyro.zeroYaw();
                Robot.estimator.setPos(AutoPathHelper.getPath(AutoPathHelper.PATH_TYPE.RIGHT_ROCKET_1).getCurrentSegment().getStartPoint());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
            
        });
        //drive from the hab to the far side of the right rocket
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_ROCKET_1));
        //place the hatch

        //back up and go to a turnaround point
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_ROCKET_2));
        //finish turning around and drive to the exchange
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_ROCKET_3));
        //pick up the new hatch

        //drive backwards to a turnaround point
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_ROCKET_4));
        //drive to the near side of the right rocket
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_ROCKET_5));
        //place the second hatch
    }
}