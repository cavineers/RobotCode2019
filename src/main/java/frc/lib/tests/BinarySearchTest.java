package frc.lib.tests;

import java.util.ArrayList;

public class BinarySearchTest {

    static ArrayList<Double> map = new ArrayList<Double>();
    public static void main(String[] args) {
        map.add(1.0);
        map.add(0.9);
        map.add(0.8);
        map.add(0.7);
        map.add(0.6);
        map.add(0.5);
        map.add(0.4);
        map.add(0.3);
        map.add(0.2);
        map.add(0.1);
        
        System.out.println("Index: " + getMapIndexForTimestamp(0.45));
    }
    public static int getMapIndexForTimestamp(double timestamp) {
        System.out.println(timestamp);
        int first = 0;
        int last = map.size() - 1;
        int mid;
        while (first <= last) {
            mid = ( first + last ) / 2;
            if (timestamp == map.get(mid)) {
                return mid;
            } else if (map.get(mid) < timestamp) {
                last = mid - 1;
            } else {
                first = mid + 1;
            }
        }
        if (last >= 0 && first < map.size()) {
            System.out.println("lower bounds: " + map.get(first) + " index: " + first);
            System.out.println("upper bounds: " + map.get(last) + " index: " + last);
        } else if (last >= 0){
            System.out.println("smaller than any element in the array");
        } else {
            System.out.println("larger than any element in the array");
        }
        return -1;
    }

}