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
        map.addWheelUpdate(1, 1, 1);
        map.addWheelUpdate(1, 1, 2);
        map.addCameraUpdate(0, 0, 1.75);

        //TODO: check if position map updates as expected
        
        RobotPos pos = map.getFieldRelativePosition();
        System.out.println("x: " + pos.getX() + " y: " + pos.getY());
    }


}