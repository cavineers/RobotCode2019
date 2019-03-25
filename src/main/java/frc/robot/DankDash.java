package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.CheckEncoders;
import frc.robot.commands.auto.DisableAutoOverride;
import frc.robot.commands.auto.OverrideAutoSelection;

public class DankDash {
    private NetworkTable netTable;
    private String profileName;
    private String profileLocation;
    private CheckEncoders encoderCheck;
    private boolean coastBreakToggle;
    private CoastBreakState cbState;

    private enum CoastBreakState {
        COAST, BREAK
    }

    public DankDash() {
        // Nettables
        this.netTable = NetworkTableInstance.getDefault().getTable("DankDash");
    }

    public void setProfileName(String profileName) {
        this.profileName = profileName;
    }

    public void addListener() {
        netTable.addEntryListener((table, key, entry, value, flags) -> {
            System.out.println("Key: " + key);
            if (key == "encoderCheck") {
                if (value.getBoolean()) {
                    this.encoderCheck = new CheckEncoders();
                    netTable.getEntry("encoderCheck").setBoolean(false);
                }
            }
            if (key == "toggleCoastBreak") {
                this.toggleBreakCoast();
            }
            if (key == "OverrideAuto") {
                if (value.getBoolean()) {
                    new OverrideAutoSelection("DANKDASH");
                } else {
                    new DisableAutoOverride();
                }
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
    }

    public String getProfileName() {
        return this.profileName;
    }

    public void setProfileLocation(String profileLocation) {
        this.profileLocation = profileLocation;
    }

    public String getProfileLocation() {
        return this.profileLocation;
    }

    public void export() {
        netTable.getEntry("ProfileData").setString(this.profileLocation);
        netTable.getEntry("ProfileName").setString(this.profileName);
    }

    public void sendDash(String id, String data) {
        netTable.getEntry(id).setString(data);
    }

    public void enableCoastBreakToggle() {
        this.coastBreakToggle = true;
    }

    public void disableCoastBreakToggle() {
        this.coastBreakToggle = false;
    }

    public boolean getCoastBreakToggle() {
        return this.coastBreakToggle;
    }

    private void toggleBreakCoast() {
        System.out.println("[DANKDASH] Toggling coast/break on elevator & grabber");
        if (cbState == CoastBreakState.BREAK) {
            Robot.elevator.elevatorMotor.setIdleMode(IdleMode.kCoast);
            Robot.grabber.getArmMotor().setIdleMode(IdleMode.kCoast);
            cbState = CoastBreakState.COAST;
        } else {
            Robot.elevator.elevatorMotor.setIdleMode(IdleMode.kBrake);
            Robot.grabber.getArmMotor().setIdleMode(IdleMode.kBrake);
            cbState = CoastBreakState.BREAK;
        }
    }
}