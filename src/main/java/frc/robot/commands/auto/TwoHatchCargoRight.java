package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.AutoPathHelper;
import frc.robot.Robot;
import frc.robot.commands.DriveForward;
import frc.robot.commands.FollowPath;
import frc.robot.commands.HomeAll;
import frc.robot.commands.TargetVisionTape;
import frc.robot.commands.elevator.ElevatorToGround;
import frc.robot.commands.grabber.ChangeGrabberState;
import frc.robot.commands.grabber.ChangeHatchGrabberState;
import frc.robot.commands.grabber.HomeGrabber;
import frc.robot.subsystems.Grabber.GrabberPosition;
import frc.robot.subsystems.Grabber.HatchGrabberState;

public class TwoHatchCargoRight extends CommandGroup{
    public TwoHatchCargoRight() {
        addSequential(new Command() {
            @Override
            protected void initialize() {
                System.out.println("starting...");
                Robot.gyro.zeroYaw();
                Robot.estimator.setPos(AutoPathHelper.getPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_1).getCurrentSegment().getStartPoint());
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
            
        });
        addSequential(new HomeAll());
        //drive from the hab to the left side of the cargo bay front
        addSequential(new FollowPath(AutoPathHelper.PATH_TYPE.RIGHT_CARGOBAY_1));
        //place the hatch
        addSequential(new DriveForward(0.7,0.3));
    }
}