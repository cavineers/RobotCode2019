package frc.robot.dashcomponents;

public class VideoStream {
    private String path;
    private int loc_x;
    private int loc_y;
    private int loc_z;
    private String width;
    private String height;

    public VideoStream() {
        
    }

    public String getPath() {
        return this.path;
    }

    public void setPath(String path) {
        this.path = path;
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

    public String getWidth() {
        return width;
    } 

    public void setWidth(String width) {
        this.width = width;
    }

    public String getHeight() {
        return height;
    }

    public void setHeight(String height) {
        this.height = height;
    }

    public String getJson(boolean addComma) {
        if (addComma) {
            return "{\"type\": \"VideoStream\", \"path\": \""+path+"\", \"x\": \""+loc_x+"\", \"y\": \""+loc_y+"\", \"z\": \""+loc_z+"\", \"width\": \""+width+"\", \"height\": \""+height+"\"},";
        } else  {
            return "{\"type\": \"VideoStream\", \"path\": \""+path+"\", \"x\": \""+loc_x+"\", \"y\": \""+loc_y+"\", \"z\": \""+loc_z+"\", \"width\": \""+width+"\", \"height\": \""+height+"\"}";
        }
    }
}