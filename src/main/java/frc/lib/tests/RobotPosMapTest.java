package frc.lib.tests;

import java.util.ArrayList;

import frc.lib.RobotPos;
import frc.lib.RobotPosMap;
import frc.lib.RobotPosUpdate;
import frc.lib.RobotPosUpdate.UpdateType;

public class RobotPosMapTest {

    public static RobotPosMap map;
    public static void main(String[] args) {
        map = new RobotPosMap(10, new RobotPosUpdate(0,0, 0, UpdateType.BASE));
        map.addWheelUpdate(5, 5, 1);
        map.addWheelUpdate(2, 10, 2);
        map.addWheelUpdate(1, 1, 3);
        map.addWheelUpdate(1, 1, 4);
        map.addWheelUpdate(1, 1, 5);
        map.addWheelUpdate(1, 1, 6);
        map.addWheelUpdate(1, 1, 7);
        map.addWheelUpdate(1, 1, 8);
        map.addWheelUpdate(1, 1, 9);
        map.addWheelUpdate(1, 1, 10);
        map.addWheelUpdate(1, 1, 11);
        // map.addCameraUpdate(0, 0, 4.5);

        //check if position map updates as expected

        RobotPos pos = map.getFieldRelativePosition();
        System.out.println("x: " + pos.getX() + " y: " + pos.getY());
        map.printMap();
    }


}