package frc.robot;

// import frc.robot.dashcomponents.*;
import edu.wpi.first.networktables.*;
import frc.robot.commands.CheckEncoders;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

// ===== Available Components =====
//
// Button
// Checkbox
// Container
// Heartbeat
// Image
// Refresh button
// Static Text
// Updating Text
// Video stream

public class DankDash {
    private NetworkTable netTable;
    private String profileName;
    private String profileLocation;
    private CheckEncoders encoderCheck;
    private boolean coastBreakToggle;
    private CoastBreakState cbState;

    private enum CoastBreakState {
        COAST,
        BREAK
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
                    encoderCheck = new CheckEncoders();
                    netTable.getEntry("encoderCheck").setBoolean(false);
                }
            }
            if (key == "toggleCoastBreak") {
                this.toggleBreakCoast();
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