package frc.lib.dubinPath.geometry;

import java.util.ArrayList;

import frc.lib.pathPursuit.Point;

//import android.util.Pair;

public class Geometry {
    // Constants ////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    
    public static final double PI = 3.14159265359;
    
    
    // Classes //////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
    
    public static class Pair<F,S> {
        public final F first;
        public final S second;
        
        public Pair() {
            this(null,null);
        }
        
        public Pair(F first_, S second_) {
            this.first = first_;
            this.second = second_;
        }
        
        public static <A,B> Pair<A,B> create(A a,B b) {
            return new Pair<A,B>(a,b);
        }
    }
    
    /////////////////////////////////////////////////////////////
    
    public static class Point2d extends Pair<Double,Double> {
        public Point2d() {
            super(0.0,0.0);
        }

        public Point2d(double first, double second) {
            super(first, second);
        }
        
        public static Point2d create(double a,double b) {
            return new Point2d(a,b);
        }

        public Point toPoint() {
            return new Point(this.first, this.second);
        }
    }
    
    /////////////////////////////////////////////////////////////
    
    public static class Pose2d {
        public Point2d pos;
        public final double theta;
        
        public Pose2d(){
            pos = new Point2d(0.0,0.0);
            theta = 0.0;
        }
        
        public Pose2d(Double posx, Double posy, Double theta) {
            this.pos   = new Point2d(posx,posy);
            this.theta = theta;
        }
        
    }
    
    /////////////////////////////////////////////////////////////
    
    public static class Circle {
        private double[] values_;

        public Circle() {
            values_ = new double[3];
            values_[0] = values_[1] = values_[2] = 0.0;
        }

        public Circle(float x, float y, float r){
            values_ = new double[3];
            values_[0] = x;
            values_[1] = y;
            values_[2] = r;
        }

        public Circle(double x, double y, double r){
            values_ = new double[3];
            values_[0] = x;
            values_[1] = y;
            values_[2] = r;
        }

        public void SetPos(Pair<Double, Double> pos) {
            values_[0] = pos.first;
            values_[1] = pos.second;
        }

        public void SetPos(double x, double y) {
            values_[0] = x;
            values_[1] = y;
        }

        public void SetRadius(double radius){
            values_[2] = radius;
        }

        public Point2d GetPos() {
            return new Point2d(values_[0],values_[1]);
        }

        public double GetX(){
            return values_[0];
        }

        public double GetY() {
            return values_[1];
        }

        public double GetRadius() {
            return values_[2];
        }
    }
    
    
    // Methods //////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    public static double Norm2d(Pair<Double, Double> lhs, Pair<Double, Double> rhs){
        return Math.sqrt((rhs.first-lhs.first)*(rhs.first-lhs.first) + (rhs.second - lhs.second)*(rhs.second - lhs.second));
    }
    
    /////////////////////////////////////////////////////////////

    public static double Norm2d(Pair<Double, Double> vector) {
        return Math.sqrt(vector.first*vector.first + vector.second*vector.second);
    }
    
    /////////////////////////////////////////////////////////////

    public static ArrayList<Pair<Point2d,Point2d>> TangentLines(Circle c1, Circle c2) {
        double x1 = c1.GetX();
        double y1 = c1.GetY();
        double x2 = c2.GetX();
        double y2 = c2.GetY();
        double r1 = c1.GetRadius();
        double r2 = c2.GetRadius();
        double d_sq = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
        ArrayList<Pair<Point2d,Point2d> > returnVec = new ArrayList<Pair<Point2d,Point2d>>();
        if (d_sq < (r1-r2)*(r1-r2)){
            //we may have a problem, the circles are either intersecting, one is within the other, but still tangent
            //at one point, or one is completely in the other. We only have a problem if one is within the other, but
            //not tangent to it anywhere
            if (d_sq != Math.max(r1,r2) && d_sq < Math.max(r1,r2)){
                return returnVec;
            }//else they are intersecting or one is within the other, but still tangent to it
            //in the other two cases, either 1 or 2 external tangent lines remain, but there are no internal tangent
            //lines
        }

        double d = Math.sqrt(d_sq); 
        double vx = (x2 - x1) / d;
        double vy = (y2 - y1) / d;
        for (int sign1 = +1; sign1 >= -1; sign1 -= 2) {
            double c = (r1 - sign1 * r2) / d;
            if (c*c > 1.0) continue; //want to be subtracting small from large, not adding
            double h = Math.sqrt(Math.max(0.0, 1.0 - c*c));

            for (int sign2 = +1; sign2 >= -1; sign2 -= 2) {
                double nx = vx * c - sign2 * h * vy;
                double ny = vy * c + sign2 * h * vx;
                returnVec.add(new Pair<Point2d,Point2d>(new Point2d(x1 + r1 * nx, y1 + r1 * ny), new Point2d(x2 + sign1 * r2 * nx, y2 + sign1 * r2 * ny)));
            }
        }
        return returnVec;
    }//end TangentLines function
    
    /////////////////////////////////////////////////////////////
    
    //center point, startPoint, endPoint
    public static double ArcLength(Point2d center, Point2d lhs, Point2d rhs, double radius, boolean left){
          //ArcLength is defined as the radius of the circle * theta, the angle between the two points along the
          //circumference

          //generally, you can find the short angle between the points given the circle's center point if you turn the
          //points on the circumference into vectors from the circle center. Using the dot product of the vectors, we
          //can determine the angle between them.
          //However, for Dubin's cars we need to know directional information, and acos() only gives us a range of
          //[0,PI] radians. Because circles for the Dubins cars are either right or left-turn only circles, we need to
          //know the angle between the two points, if we were only traveling the circle's direction (left or right).
          //Using atan2, which gives us [-PI, PI] range, we can get a positive or negative angle between our start
          //(_lhs) and goal (_rhs) points. atan2(goal) - atan2(start) will give us a positive angle if, going from
          //start to goal we must rotate through a positive angle (regardless of circle direction). Atan2 still only
          //gives us the short angle, but the directional information is useful. We can say that if the returned angle
          //to rotate through is negative (right turn) but the circle's direction is positive (left turn), we'd rather
          //have the larger angle (2PI - abs(angle_returned). Vice versa for positive angles in a right-turn circle.

          Point2d vec1 = new Point2d(lhs.first - center.first, lhs.second - center.second);
          Point2d vec2 = new Point2d(rhs.first - center.first, rhs.second - center.second);
          
          double theta = Math.atan2(vec2.second, vec2.first) - Math.atan2(vec1.second,vec1.first);
          if (theta < -1e-6 && left)
            theta += 2.0*PI;
          else if (theta > 1e-6 && !left)
            theta -= 2.0 * PI;

          return Math.abs(theta*radius);
        }//end ArcLength
    
    /////////////////////////////////////////////////////////////
}