package frc.robot.commands.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Point;
import frc.lib.pathPursuit.Segment;
import frc.robot.Robot;
import frc.robot.commands.HomeAll;

public class VCargo extends CommandGroup {
    private boolean finished = false;
    private NetworkTable netTable;
    Path path;

    public enum VCargoHeight {
        CARGO_SHIP, ROCKET_LVL1, ROCKET_LVL2, ROCKET_LVL3;
    }

    public VCargo() {
        System.out.println("Running VCargo");
        Robot.isAutoOverridden = false;
        this.netTable = NetworkTableInstance.getDefault().getTable("Vision"); // Get the vision network table

        addSequential(new HomeAll()); // Home all motors

        // addSequential(new DriveForward(0.5,9.0)); // Drive forward test
        Robot.drivetrain.drive(0.5, 0);
        path = new Path();
        path.addSegment(new LineSegment(new Point(0, 0), new Point(100, 0), 60, 0));
    }

	@Override
	protected boolean isFinished() {
		return finished;
	}
}