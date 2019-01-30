package frc.lib.tests;

import frc.lib.Vector3D;

public class VectorTest {

    public static void main(String args[]) {
        Vector3D vect = new Vector3D(0.25, 0.5, 1);
        vect = vect.rotateZAxis(-Math.PI/2);
        vect = vect.rotateYAxis(Math.PI);
        System.out.println(vect);
    }
}