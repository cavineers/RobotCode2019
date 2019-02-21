package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.commands.DriveForward;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

public class CheckEncoders extends CommandGroup {
    private NetworkTable netTable;

    public CheckEncoders() {
        this.netTable = NetworkTableInstance.getDefault().getTable("DankDash");
        addSequential(new Command() {
            @Override
            protected void initialize() {
                Robot.drivetrain.getRightTalon().setSelectedSensorPosition(0, 0, 0);
                Robot.drivetrain.getLeftTalon().setSelectedSensorPosition(0, 0, 0);
            }

            @Override
            protected boolean isFinished() {
                return true;
            }
        });

        addSequential(new DriveForward(0.15, 0.3));
        addSequential(new Command() {
            @Override
            protected void initialize() {
                int right = Robot.drivetrain.getRightTalon().getSelectedSensorPosition(0);
                int left = Robot.drivetrain.getLeftTalon().getSelectedSensorPosition(0);
                if (right == 0 && left == 0) {
                    netTable.getEntry("ENCODER STATUS").setString("BOTH ERROR");
                } else if (right == 0) {
                    netTable.getEntry("ENCODER STATUS").setString("RIGHT ERROR");
                } else if (left == 0) {
                    netTable.getEntry("ENCODER STATUS").setString("LEFT ERROR");
                } else {
                    netTable.getEntry("ENCODER STATUS").setString("NOMINAL");
                }
            }

            @Override
            protected boolean isFinished() {
                return true;
            }

        });
    }
}