package frc.robot.dashcomponents;

public class HeartbeatService {
    private String ntLocation;

    public HeartbeatService(String ntLocation) {
        this.ntLocation = ntLocation;
    }

    public void setLocation(String ntLocation) {
        this.ntLocation = ntLocation;
    }

    public String getLocation() {
        return this.ntLocation;
    }

    public String getJson(boolean addComma) {
        if (addComma) {
            return "{\"type\": \"HeartbeatService\", \"ntLocation\": \""+ntLocation+"\"},";
        } else {
            return "{\"type\": \"HeartbeatService\", \"ntLocation\": \""+ntLocation+"\"}";
        }
    }
}