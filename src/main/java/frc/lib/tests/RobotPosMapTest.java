package frc.lib.tests;

import java.util.ArrayList;

import frc.lib.RobotPos;
import frc.lib.RobotPosMap;
import frc.lib.RobotPosUpdate;
import frc.lib.RobotPosUpdate.UpdateType;

public class RobotPosMapTest {

    public static RobotPosMap map;
    public static void main(String[] args) {
        map = new RobotPosMap(15, new RobotPosUpdate(0,0,0, 0, UpdateType.BASE));
        map.addWheelUpdate(1, 1, 0, 1);
        map.addWheelUpdate(1, 1, 0, 2);
        map.addWheelUpdate(1, 1, 0, 3);
        map.addWheelUpdate(1, 1, 0, 4);
        // map.addWheelUpdate(1, 1, 0, 5);
        // map.addWheelUpdate(1, 1, 1, 6);
        // map.addWheelUpdate(10, 10, 2, 7);
        // map.addWheelUpdate(10, 1, 0, 8);
        // map.addWheelUpdate(1, 1, 0, 9);
        // map.addWheelUpdate(1, 1, 0, 10);
        // map.addWheelUpdate(1, 1, 0, 11);

        map.addCameraUpdate(2.9, 2.9, 0, 3.75);

        RobotPos pos = map.getLastestFieldRelativePosition();
        System.out.println("x: " + pos.getX() + " y: " + pos.getY() + " theta: " + pos.getHeading());
        map.printMap();
    }


}