package frc.lib.tests;

import frc.lib.Vector3D;
import frc.robot.Constants;

public class VectorTest {

    public static void main(String args[]) {
        Vector3D vect = new Vector3D(1, 2, 3);
        vect = vect.rotateZAxis(Math.PI/2);
        vect = vect.rotateYAxis(Math.PI);

        // vect = Vector3D.rotate(vect, Constants.kCameraToRobotMatrix);
        System.out.println(vect);
    }
}