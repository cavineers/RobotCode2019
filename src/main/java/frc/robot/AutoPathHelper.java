package frc.robot;

import frc.lib.pathPursuit.Path;
import frc.lib.pathPursuit.Segment;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.Point;

public class AutoPathHelper {

    public static enum START_POS {
        RIGHT,
        LEFT,
        MIDDLE,
        INVALID
    };

    public static enum PATH_TARGET {
        FRONT_CARGOBAY,
        SIDE_CARGOBAY,
        ROCKET
    }
    public static enum PATH_TYPE {
        TEST_PATH, TEST_PATH_CURVE,TEST_PATH_REVERSE,

        //Auto Paths for starting on the left of the hab and placing hatches to the cargo bay
        RIGHT_CARGOBAY_1, // start on the right side of the hab and allign in front of the right side of cargo bay front
        RIGHT_CARGOBAY_2, // back up from hatch placement and reverse to a turn around point
        RIGHT_CARGOBAY_3, // go forward from the turnaround point to the front of the right loading station to get a hatch
        RIGHT_CARGOBAY_4, // back up from the loading station and go back to the turning point
        RIGHT_CARGOBAY_5, // go forward from turnaround and allign in front of the left side of the cargo bay front

        //Auto Paths for starting on the right of the hab and placing hatches to the cargo bay
        LEFT_CARGOBAY_1, // start on the left side of the hab and allign in front of the left side of cargo bay front
        LEFT_CARGOBAY_2, // back up from hatch placement and reverse to a turn around point
        LEFT_CARGOBAY_3, // go forward from the turnaround point to the front of the left loading station to get a hatch
        LEFT_CARGOBAY_4, // back up from the loading station and go back to the turning point
        LEFT_CARGOBAY_5, // go forward from turnaround and allign in front of the right side of the cargo bay front

        //Auto Paths for starting on the right of the hab and placing hatches to the right rocket
        RIGHT_ROCKET_1, // start on the right side of the hab and allign in front of the far side of the right rocket
        RIGHT_ROCKET_2, // back up from hatch placement and reverse to a turn around point
        RIGHT_ROCKET_3, // go forward from the turnaround point to the front of the right loading station to get a hatch
        RIGHT_ROCKET_4, // back up from the loading station and go back to the turning point
        RIGHT_ROCKET_5, // go forward from turnaround and allign in front of the near side of the right rocket

        //Auto Paths for starting on the left of the hab and placing hatches to the left rocket
        LEFT_ROCKET_1, // start on the left side of the hab and allign in front of the far side of the left rocket
        LEFT_ROCKET_2, // back up from hatch placement and reverse to a turn around point
        LEFT_ROCKET_3, // go forward from the turnaround point to the front of the left loading station to get a hatch
        LEFT_ROCKET_4, // back up from the loading station and go back to the turning point
        LEFT_ROCKET_5, // go forward from turnaround and allign in front of the near side of the left rocket
    }

    public static Path getPath(PATH_TYPE pathType) {
        Path path;
        switch (pathType) {
        case TEST_PATH: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(0, 0), new Point(100, 0), 60, 0);
            path.addSegment(seg1);
            return path;
        } case TEST_PATH_REVERSE: {
            path = new Path(false, true, Constants.kMaxAccelSpeedUp);
            Segment seg1 = new LineSegment(new Point(0, 0), new Point(-60, 0), -30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(-60, 0), new Point(-90, -30), new Point(-60, -30), -30,0);
            path.addSegment(seg2);
            return path;
        } case TEST_PATH_CURVE: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 30, 30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, -30), new Point(60, -30), 30);
            path.addSegment(seg2);
            Segment seg3 = new ArcSegment(new Point(90, -30), new Point(120, -60), new Point(120, -30), 30);
            path.addSegment(seg3);
            Segment seg4 = new LineSegment(new Point(120, -60), new Point(170, -60), 30, 0);
            path.addSegment(seg4);
            return path;
        } 

        case RIGHT_CARGOBAY_1: {
            path = new Path(true, false, Constants.kMaxAccelSpeedUp);
            Segment seg1 = new LineSegment(new Point(66.867, 207.362), new Point(107.602, 207.362), 80, 80);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(107.602, 207.362), new Point(135.371, 190.238), new Point(107.602,176.284), 80);
            path.addSegment(seg2);
            Segment seg3 = new ArcSegment(new Point(135.371, 190.238), new Point(162.805, 173.114), new Point(162.805, 203.652), 80);
            path.addSegment(seg3);
            Segment seg4 = new LineSegment(new Point(162.805, 173.114), new Point(203.55,173.114)/*new Point(182.866, 173.114)*/, 80, 0);
            path.addSegment(seg4);
            return path;
        } case RIGHT_CARGOBAY_2: {
            path = new Path(false, true, Constants.kMaxAccelSpeedUp);
            Segment seg1 = new LineSegment(new Point(203.55,173.114), new Point(179, 173.114), -80);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(179.442, 173.114), new Point(172.393, 253.693), new Point(179.442, 213.712), -80);
            path.addSegment(seg2);
            Segment seg3 = new LineSegment(new Point(172.393, 253.693), new Point(184.21, 255.777), -80, 0);
            path.addSegment(seg3);
            return path;
        } case RIGHT_CARGOBAY_3: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(184.21, 255.777), new Point(139.929,247.789), 80);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(139.929,247.789), new Point(103.227, 272.151), new Point(134.243, 279.051), 80);
            path.addSegment(seg2);
            Segment seg3 = new ArcSegment(new Point(103.227, 272.151), new Point(71.28, 297.772), new Point(71.28, 265.044), 80);
            path.addSegment(seg3);
            Segment seg4 = new LineSegment(new Point(71.28, 297.772), new Point(18.866,297.772),/*new Point(37.435, 297.772),*/ 80, 0);
            path.addSegment(seg4);
            return path;
        } case RIGHT_CARGOBAY_4: {
            path = new Path(false, true, Constants.kMaxAccelSpeedUp);
            Segment seg1 = new LineSegment(new Point(18.866,297.772), new Point(71.28, 297.772), -80);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(71.28, 297.772), new Point(103.227, 272.151), new Point(71.28, 265.044), -80);
            path.addSegment(seg2);
            Segment seg3 = new ArcSegment(new Point(103.227, 272.151), new Point(139.929,247.789), new Point(134.243, 279.051), -80);
            path.addSegment(seg3);
            Segment seg4 = new LineSegment(new Point(139.929,247.789), new Point(184.21, 255.777), -80, 0);
            path.addSegment(seg4);
            return path;
        } case RIGHT_CARGOBAY_5: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(184.21, 255.777), new Point(162.943, 251.909), 80, 80);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(162.943, 251.909), new Point(172.032, 151.137), new Point(172.032, 201.933), 80);
            path.addSegment(seg2);
            Segment seg3 = new LineSegment(new Point(172.032, 151.137), new Point(183.955, 151.137), 80, 0);
            path.addSegment(seg3);
            return path;
        } 
        
        case LEFT_CARGOBAY_1: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_CARGOBAY_1));
        } case LEFT_CARGOBAY_2: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_CARGOBAY_2));
        } case LEFT_CARGOBAY_3: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_CARGOBAY_3));
        } case LEFT_CARGOBAY_4: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_CARGOBAY_4));
        } case LEFT_CARGOBAY_5: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_CARGOBAY_5));
        }

        case RIGHT_ROCKET_1: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(66.867, 207.362), new Point(155.107, 207.362), 30, 30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(155.107, 207.362), new Point(185.846, 227.809), new Point(155.107, 240.692), 30);
            path.addSegment(seg2);
            Segment seg3 = new ArcSegment(new Point(185.846, 227.809), new Point(243.943, 233.966), new Point(216.586, 214.927), 30);
            path.addSegment(seg3);
            Segment seg4 = new ArcSegment(new Point(243.943, 233.966), new Point(287.356, 282.213), new Point(271.299, 253.006), 30, 30, ArcSegment.TURN.LARGE_SIDE);
            path.addSegment(seg4);
            Segment seg5 = new LineSegment(new Point(287.356, 282.213), new Point(276.84, 287.994), 30, 0);
            path.addSegment(seg5);
            return path;
        } case RIGHT_ROCKET_2: {
            path = new Path(false, true, Constants.kMaxAccelSpeedUp);
            Segment seg1 = new LineSegment(new Point(261.146, 296.622), new Point(280.288, 286.132), 30, 30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(280.288, 286.132), new Point(264.871, 226.321), new Point(264.871, 258.198), 30);
            path.addSegment(seg2);
            Segment seg3 = new LineSegment(new Point(264.871, 226.321), new Point(200.931, 226.321), 30);
            path.addSegment(seg3);
            Segment seg4 = new ArcSegment(new Point(200.931, 226.321), new Point(149.224, 174.615), new Point(200.931, 174.615), 30);
            path.addSegment(seg4);
            Segment seg5 = new LineSegment(new Point(149.224, 174.615), new Point(149.224, 162.615), 30, 0);
            path.addSegment(seg5);
            return path;
        } case RIGHT_ROCKET_3: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(149.224, 162.615), new Point(149.224, 220.28), 30, 30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(149.224, 220.28), new Point(71.589, 297.916), new Point(71.589, 220.28), 30);
            path.addSegment(seg2);
            Segment seg3 = new LineSegment(new Point(71.589, 297.916), new Point(36.316, 297.916), 30, 0);
            path.addSegment(seg3);
            return path;
        } case RIGHT_ROCKET_4: {
            path = new Path(false, true, Constants.kMaxAccelSpeedUp);
            Segment seg1 = new LineSegment(new Point(18.859, 297.916), new Point(71.589, 297.916), 30, 30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(71.589, 297.916), new Point(149.224, 220.28), new Point(71.589, 220.28), 30);
            path.addSegment(seg2);
            Segment seg3 = new LineSegment(new Point(149.224, 220.28), new Point(149.224, 162.615), 30, 0);
            path.addSegment(seg3);
            return path;
        } case RIGHT_ROCKET_5: {
            path = new Path();
            Segment seg1 = new LineSegment(new Point(149.224, 162.615), new Point(149.224, 242.914), 30, 30);
            path.addSegment(seg1);
            Segment seg2 = new ArcSegment(new Point(149.224, 242.914), new Point(172.13, 281.886), new Point(193.83, 242.914), 30);
            path.addSegment(seg2);
            Segment seg3 = new LineSegment(new Point(172.13, 281.886), new Point(182.614, 287.742), 30, 0);
            path.addSegment(seg3);
            return path;
        }

        case LEFT_ROCKET_1: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_ROCKET_1));
        } case LEFT_ROCKET_2: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_ROCKET_2));
        } case LEFT_ROCKET_3: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_ROCKET_3));
        } case LEFT_ROCKET_4: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_ROCKET_4));
        } case LEFT_ROCKET_5: {
            return Path.mirror(getPath(PATH_TYPE.RIGHT_ROCKET_5));
        }

        default: {
            return null;
        }
    }
}

}