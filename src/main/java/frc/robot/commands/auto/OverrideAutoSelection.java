package frc.robot.commands.auto;

import frc.robot.AutoPathHelper.PATH_TARGET;
import frc.robot.AutoPathHelper.START_POS;
import frc.robot.Robot;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.command.Command;

public class OverrideAutoSelection extends Command {
    private String source;
    private NetworkTable netTable;

    public OverrideAutoSelection(String source) {
        this.setRunWhenDisabled(true);
        this.source = source;
        this.netTable = NetworkTableInstance.getDefault().getTable("DankDash");
    }

    @Override
    protected void initialize() {
        if (this.source == "DANKDASH") {
            Robot.isAutoOverridden = true;
            if (netTable.getEntry("/DankDash/PATH_TARGET").getString("") == "FRONT_CARGOBAY") {
                Robot.overriddenPathTarget = PATH_TARGET.FRONT_CARGOBAY;
            }
            if (netTable.getEntry("/DankDash/PATH_TARGET").getString("") == "SIDE_CARGOBAY") {
                Robot.overriddenPathTarget = PATH_TARGET.SIDE_CARGOBAY;
            }
            if (netTable.getEntry("/DankDash/PATH_TARGET").getString("") == "ROCKET") {
                Robot.overriddenPathTarget = PATH_TARGET.ROCKET;
            }
            /* */
            if (netTable.getEntry("/DankDash/START_POS").getString("") == "LEFT") {
                Robot.overriddenStartPos = START_POS.LEFT;
            }
            if (netTable.getEntry("/DankDash/START_POS").getString("") == "MIDDLE") {
                Robot.overriddenStartPos = START_POS.MIDDLE;
            }
            if (netTable.getEntry("/DankDash/START_POS").getString("") == "RIGHT") {
                Robot.overriddenStartPos = START_POS.RIGHT;
            }
        } else if (this.source == "SMARTDASH") {
            Robot.isAutoOverridden = true;
            Robot.overriddenPathTarget = Robot.targetChooser.getSelected();
            Robot.overriddenStartPos = Robot.posChooser.getSelected();
        } else {
            System.out.println("Unknown update source");
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

}