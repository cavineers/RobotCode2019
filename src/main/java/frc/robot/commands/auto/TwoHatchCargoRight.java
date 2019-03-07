package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoPathHelper;
import frc.robot.Robot;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TargetVisionTape;
import frc.robot.commands.elevator.ElevatorToGround;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.commands.grabber.ChangeHatchGrabberState;
import frc.robot.commands.grabber.HomeGrabber;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.HatchGrabberState;

public class TwoHatchCargoRight extends CommandGroup{
    public TwoHatchCargoRight() {
        //initial homing routines
        // addSequential(new HomeGrabber());
        // addSequential(new ElevatorToGround());
        // addSequential(new ChangeGrabberState(GrabberPosition.EXTENDED));
        //make sure that the hatch grabber holds in the hatch
        addSequential(new Command() {
            @Override
            protected void initialize() {
                Robot.gyro.zeroYaw();
                Robot.estimator.setPos(AutoPathHelper.getPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_1).getCurrentSegment().getStartPoint());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
            
        });
        // addSequential(new ChangeHatchGrabberState(HatchGrabberState.OPEN));
        
        //drive from the hab to the right side of the cargo bay front
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_1));
        //allign to be with the tape

        // addSequential(new TargetVisionTape());
        //make sure we are flush with the wall by driving forward for a bit
        addSequential(new DriveForward(0.5, 0.5));
        //close the hatch grabber to place the hatch
        // addSequential(new ChangeHatchGrabberState(HatchGrabberState.CLOSED));

        //back up and go to a turnaround point
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_2));
        //finish turning around and drive to the exchange
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_3));
        //pick up the new hatch

        // addSequential(new TargetVisionTape());
        //make sure we are flush with the wall by driving forward for a bit
        // addSequential(new DriveForward(0.5, 0.5));
        //open the hatch grabber to hold on to the hatch
        // addSequential(new ChangeHatchGrabberState(HatchGrabberState.OPEN));

        //drive backwards to a turnaround point
        // addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_4));
        //drive to in front of the left side of the cargo bay
        // addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_5));
        // addSequential(new DriveForward(0.5, 0.5));
        //place the second hatch
    }
}