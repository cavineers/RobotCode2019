package frc.lib.tests;

import frc.lib.Vector3D;

public class VectorTest {

    public static void main(String args[]) {
        Vector3D vect = new Vector3D(0, 1, 0);
        vect = vect.rotateXAxis(Math.PI/2);
        System.out.println(vect);
    }
}