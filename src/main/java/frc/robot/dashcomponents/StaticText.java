package frc.robot.dashcomponents;

public class StaticText {
    private String text;
    private int loc_x;
    private int loc_y;
    private int loc_z;
    private String textColor;

    public StaticText(String text) {
        this.text = text;
    }

    public int getXPos() {
        return loc_x;
    }

    public void setXPos(int loc_x) {
        this.loc_x = loc_x;
    }

    public int getYPos() {
        return this.loc_y;
    }

    public void setYPos(int loc_y) {
        this.loc_y = loc_y;
    }

    public int getZPos() {
        return loc_z;
    }

    public void setZPos(int loc_z) {
        this.loc_z = loc_z;
    }

    public void setTextColor(String textcolor) {
        this.textColor = textcolor;
    }

    public String getTextColor() {
        return textColor;
    }

    public String getJson(boolean addComma) {
        if (addComma) {
            return "{\"type\": \"StaticText\", \"textColor\": \""+textColor+"\", \"value\": \""+text+"\", \"x\": \""+loc_x+"\", \"y\": \""+loc_y+"\", \"z\": \""+loc_z+"\"},";
        } else {
            return "{\"type\": \"StaticText\", \"textColor\": \""+textColor+"\", \"value\": \""+text+"\", \"x\": \""+loc_x+"\", \"y\": \""+loc_y+"\", \"z\": \""+loc_z+"\"}";
        }
    }
}