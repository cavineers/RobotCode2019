package frc.lib.dubinPath;

import frc.lib.pathPursuit.Path;

public class DubinsPath {
    Path path;
    double radius = 0;
    boolean isValid = false;

    public DubinsPath(Path path, double radius, boolean isValid) {
        this.path = path;
        this.radius = radius;
        this.isValid = isValid;
    }

    public Path getPath() {
        return path;
    }
    
    public double getRadius() {
        return radius;
    }

    public boolean isValid() {
        return true;
        // return isValid;
    }
}