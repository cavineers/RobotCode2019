package frc.robot.dashcomponents;

public class Button {
    private String loc_x;
    private String loc_y;
    private String loc_z;
    private String label;
    private String textColor;
    private String backgroundColor;

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

    public void setTextColor(String textColor) {
        this.textColor = textColor;
    }

    public String getTextColor() {
        return this.textColor;
    }

    public void setBackgroundColor(String bgcolor) {
        this.backgroundColor = bgcolor;
    }

    public String getBackgroundColor() {
        return this.backgroundColor;
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public String getLabel() {
        return this.label;
    }

    public String getJson(boolean addComma) {
        if (addComma) {
            return "{\"type\": \"Button\", \"x\": \""+loc_x+"\", \"y\": \""+loc_y+"\", \"z\": \""+loc_z+"\", \"bgColor\": \""+backgroundColor+"\", \"textColor\": \""+textColor+"\", \"label\": \""+label+"\"},";
        } else {
            return "{\"type\": \"Button\", \"x\": \""+loc_x+"\", \"y\": \""+loc_y+"\", \"z\": \""+loc_z+"\", \"bgColor\": \""+backgroundColor+"\", \"textColor\": \""+textColor+"\", \"label\": \""+label+"\"}";
        }
    }
}