package frc.robot.dashcomponents;

public class Checkbox {
    private String loc_x;
    private String loc_y;
    private String loc_z;
    private String label;

    public void setXPos(String loc_x) {
        this.loc_x = loc_x;
    }

    public String getXPos() {
        return this.loc_x;
    }

    public void setYPos(String loc_y) {
        this.loc_y = loc_y;
    }

    public String getYPos() {
        return this.loc_y;
    }

    public void setZPos(String loc_z) {
        this.loc_z = loc_z;
    }

    public String getZPos() {
        return this.loc_z;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public String getLabel() {
        return this.label;
    }

    public String getJson(boolean addComma) {
        return "";
    }
}