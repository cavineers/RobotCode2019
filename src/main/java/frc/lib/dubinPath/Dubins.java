package frc.lib.dubinPath;

import java.util.ArrayList;

//import android.util.Pair;

import frc.lib.dubinPath.geometry.Geometry;

import frc.lib.pathPursuit.ArcSegment;
import frc.lib.pathPursuit.LineSegment;
import frc.lib.pathPursuit.ArcSegment.TURN;
import frc.robot.Constants;

class CompareTrajectories {
    public boolean compare(DubinsTrajectory lhs, DubinsTrajectory rhs) {
        return lhs.length < rhs.length;
    }
}

public class Dubins {
    Geometry.Pose2d m_start, m_goal;
    double m_minTurnRadius;
    static final double DELTA = 0.05;

    public static void main(String[] args) {
        System.out.println("Reading args");
        Double mtr,spx,spy,spt,gpx,gpy,gpt;
        mtr = spx = spy = spt = gpx = gpy = gpt = 0.0;
        try {
            if (args.length == 4) {
                spx = 0.0;
                spy = 0.0;
                spt = 0.0;
                gpx = Double.parseDouble(args[1]);
                gpy = Double.parseDouble(args[2]);
                gpt = Double.parseDouble(args[3]);
            }
            else if (args.length == 7) {
                spx = Double.parseDouble(args[1]);
                spy = Double.parseDouble(args[2]);
                spt = Double.parseDouble(args[3]);
                gpx = Double.parseDouble(args[4]);
                gpy = Double.parseDouble(args[5]);
                gpt = Double.parseDouble(args[6]);
            }

            mtr = Double.parseDouble(args[0]);

            System.out.println("Creating Dubins instance");
            Dubins dubins = new Dubins();

            dubins.setMinTurnRadius(mtr);

            System.out.println("Executing getShortestPath");
            DubinsTrajectory DT = dubins.getShortestPath(new Geometry.Pose2d(spx,spy,spt), new Geometry.Pose2d(gpx,gpy,gpt));
            System.out.println("Trajectory:\n Type = " + DT.type + "\n Controls = \nSteering radius \t\tTime steps");
            for (int i = 0; i < DT.controls.size(); i++) {
                System.out.println( DT.controls.get(i).steeringRadius + "\t" + DT.controls.get(i).timesteps );
            }
            System.out.println("end");
        }

        catch (NumberFormatException e) {
            System.out.println("Usage: Dubins minTurnRadius startingposex startingposey startingposet goalposex goalposey goalposet\n" +
                    "       Dubins minTurnRadius goalposex goalposey goalposet");
        }
        catch (Exception e) {
            System.err.println(e.getMessage());
        }
    }

    public void setMinTurnRadius(double minTurnRadius) {
        this.m_minTurnRadius = minTurnRadius;
    }


    /** Finds the shortest Dubins Path for a given minimum turn radius, a wheelbase and a start and end position.
     * @param minTurnRadius Minimum turning radius of Dubins vehicle
     * @param wheelbase		Wheelbase of Dubins vehicle
     * @param start			Starting point of trajectory
     * @param goal			Goal point of trajectory
     * @return Shortest Dubins trajectory between the start and end point
     */
    public DubinsTrajectory getShortestPath(Geometry.Pose2d start, Geometry.Pose2d goal) throws Exception {
        Geometry.Circle agentLeft = new Geometry.Circle();
        Geometry.Circle agentRight = new Geometry.Circle();
        Geometry.Circle queryLeft = new Geometry.Circle();
        Geometry.Circle queryRight = new Geometry.Circle();
        this.m_start = start; // TODO: This might go wrong when asking for a second trajectory from the same instance of Dubins: m_start.pos is immutable...
        this.m_goal = goal; // TODO: Same here
        if ( this.m_minTurnRadius == 0.0 ) {
            throw new Exception("Minimum turn radius not set. Set before calculating shortest path!"); 
        }

        // Create circles on left and right of vehicle and on left and right of goal pose
        double theta = m_start.theta;
        theta += Geometry.PI/2.0;
        if (theta > Geometry.PI) {
            theta -= 2.0*Geometry.PI;
        }

        agentLeft.SetPos(m_start.pos.first + m_minTurnRadius*Math.cos(theta), m_start.pos.second + m_minTurnRadius*Math.sin(theta));
        agentLeft.SetRadius(m_minTurnRadius);

        theta = m_start.theta;
        theta -= Geometry.PI/2.0;
        if (theta < -Geometry.PI) {
            theta += 2.0*Geometry.PI;
        }

        agentRight.SetPos(m_start.pos.first + m_minTurnRadius*Math.cos(theta), m_start.pos.second + m_minTurnRadius*Math.sin(theta));
        agentRight.SetRadius(m_minTurnRadius);

        theta = m_goal.theta;
        theta += Geometry.PI/2.0;
        if (theta > Geometry.PI) {
            theta -= 2.0*Geometry.PI;
        }

        queryLeft.SetPos(m_goal.pos.first + m_minTurnRadius*Math.cos(theta), m_goal.pos.second + m_minTurnRadius*Math.sin(theta));
        queryLeft.SetRadius(m_minTurnRadius);

        theta = m_goal.theta;
        theta -= Geometry.PI/2.0;
        if (theta < -Geometry.PI) {
            theta += 2.0*Geometry.PI;
        }

        queryRight.SetPos(m_goal.pos.first + m_minTurnRadius*Math.cos(theta), m_goal.pos.second + m_minTurnRadius*Math.sin(theta));
        queryRight.SetRadius(m_minTurnRadius);


        // Calculate best trajectory
        DubinsTrajectory shortest = new DubinsTrajectory();
        DubinsTrajectory next = new DubinsTrajectory();

        next = BestCSCTrajectory(agentLeft,agentRight,queryLeft,queryRight);
        if (next.length < shortest.length)
            shortest = next;
        next = BestCCCTrajectory(agentLeft,agentRight,queryLeft,queryRight);
        if(next.length < shortest.length)
            shortest = next;

        return shortest;
    }

    /** BestCSCTrajectory finds the shortest Dubins trajectory using a circle segment, straight line segment and circle segment, consecutively
     * @param agentLeft		Circle to the left of the vehicle
     * @param agentRight	Circle to the right of the vehicle
     * @param queryLeft		Circle to the left of the goal
     * @param queryRight	Circle to the right of the goal
     * @return The shortest CSC Dubins trajectory
     */
    private DubinsTrajectory BestCSCTrajectory(Geometry.Circle agentLeft, Geometry.Circle agentRight, Geometry.Circle queryLeft, Geometry.Circle queryRight) {
        ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d> > RRTangents = Geometry.TangentLines(agentRight,queryRight);
        ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d> > LLTangents = Geometry.TangentLines(agentLeft,queryLeft);
        ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d> > RLTangents = Geometry.TangentLines(agentRight,queryLeft);
        ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d> > LRTangents = Geometry.TangentLines(agentLeft,queryRight);

        DubinsTrajectory shortest, next;
        shortest = new DubinsTrajectory();

        //calculate RSR
        next = RSRTrajectory(RRTangents, agentRight, queryRight);
        if (next.length < shortest.length)
            shortest = next;

        //calculate LSL
        next = LSLTrajectory(LLTangents, agentLeft, queryLeft);
        if (next.length < shortest.length)
            shortest = next;

        //calculate RSL
        next = RSLTrajectory(RLTangents, agentRight, queryLeft);
        if (next.length < shortest.length)
            shortest = next;

        //calculate LSR
        next = LSRTrajectory(LRTangents, agentLeft, queryRight);
        if (next.length < shortest.length)
            shortest = next;

        return shortest;
    }

    private DubinsTrajectory RSRTrajectory(ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d>> RRTangents, Geometry.Circle agentRight, Geometry.Circle queryRight) {
        DubinsTrajectory next = new DubinsTrajectory();
        next.type = TrajectoryType.RSR;
        double arcL1, arcL2, arcL3; //arcLengths
        Control nextControl; //for a control vector
        if (RRTangents.size() > 0){
            nextControl = new Control();
            //tangent pts function returns outer tangents for RR connection first
            nextControl.steeringRadius = -1.0 * m_minTurnRadius; //right turn at max
            arcL1 = Geometry.ArcLength(agentRight.GetPos(), m_start.pos, RRTangents.get(0).first, m_minTurnRadius, false);
            //don't use velocities because Dubins assumes unit forward velocity
            nextControl.timesteps = arcL1 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(m_start.pos.toPoint(), RRTangents.get(0).first.toPoint(), agentRight.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = 0.0; //straight
            arcL2 = Geometry.Norm2d(RRTangents.get(0).first, RRTangents.get(0).second);
            nextControl.timesteps = arcL2 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new LineSegment(RRTangents.get(0).first.toPoint(), RRTangents.get(0).second.toPoint())); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = -1.0 * m_minTurnRadius; //right turn at max
            arcL3 = Geometry.ArcLength(queryRight.GetPos(),RRTangents.get(0).second, m_goal.pos, m_minTurnRadius, false);
            nextControl.timesteps = arcL3 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(RRTangents.get(0).second.toPoint(), m_goal.pos.toPoint(), queryRight.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center

            //calculate total length
            next.length =  arcL1 + arcL2 + arcL3;
        }
        return next;
    }

    private DubinsTrajectory LSLTrajectory(ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d>> LLTangents, Geometry.Circle agentLeft, Geometry.Circle queryLeft) {
        DubinsTrajectory next = new DubinsTrajectory();
        next.type = TrajectoryType.LSL;
        double arcL1, arcL2, arcL3; //arcLengths
        if (LLTangents.size() > 1){
            Control nextControl = new Control();
            //tangent pts function returns outer tangents for LL connection second
            nextControl.steeringRadius = m_minTurnRadius; //left turn at max
            arcL1 = Geometry.ArcLength(agentLeft.GetPos(), m_start.pos, LLTangents.get(1).first, m_minTurnRadius, true);
            //don't use velocities because Dubins assumes unit forward velocity
            nextControl.timesteps = arcL1 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(m_start.pos.toPoint(), LLTangents.get(1).first.toPoint(), agentLeft.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center


            nextControl = new Control();
            nextControl.steeringRadius = 0.0; //straight
            arcL2 = Geometry.Norm2d(LLTangents.get(1).first, LLTangents.get(1).second);
            nextControl.timesteps = arcL2 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new LineSegment(LLTangents.get(1).first.toPoint(), LLTangents.get(1).second.toPoint())); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = m_minTurnRadius; //left turn at max
            arcL3 = Geometry.ArcLength(queryLeft.GetPos(),LLTangents.get(1).second, m_goal.pos, m_minTurnRadius, true);
            nextControl.timesteps = arcL3 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(LLTangents.get(1).second.toPoint(), m_goal.pos.toPoint(), queryLeft.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center

            //calculate total length
            next.length =  arcL1 + arcL2 + arcL3;
        }		
        return next;
    }

    private DubinsTrajectory RSLTrajectory(ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d>> RLTangents, Geometry.Circle agentRight, Geometry.Circle queryLeft) {
        DubinsTrajectory next = new DubinsTrajectory();
        next.type = TrajectoryType.RSL;
        double arcL1, arcL2, arcL3; //arcLengths
        Control nextControl;
        if (RLTangents.size() > 2){
            nextControl = new Control();
            //tangent pts function returns inner tangents for RL connection third 
            nextControl.steeringRadius = -1.0 * m_minTurnRadius; //right turn at max
            arcL1 = Geometry.ArcLength(agentRight.GetPos(), m_start.pos, RLTangents.get(2).first, m_minTurnRadius, false);
            //don't use velocities because Dubins assumes unit forward velocity
            nextControl.timesteps = arcL1 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(m_start.pos.toPoint(), RLTangents.get(2).first.toPoint(), agentRight.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = 0.0; //straight
            arcL2 = Geometry.Norm2d(RLTangents.get(2).first, RLTangents.get(2).second);
            nextControl.timesteps = arcL2 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new LineSegment(RLTangents.get(2).first.toPoint(), RLTangents.get(2).second.toPoint())); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = m_minTurnRadius; //left turn at max
            arcL3 = Geometry.ArcLength(queryLeft.GetPos(), RLTangents.get(2).second, m_goal.pos, m_minTurnRadius, true);
            nextControl.timesteps = arcL3 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(RLTangents.get(2).second.toPoint(), m_goal.pos.toPoint(), queryLeft.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center

            //calculate total length
            next.length =  arcL1 + arcL2 + arcL3;
        }
        return next;
    }

    private DubinsTrajectory LSRTrajectory(ArrayList<Geometry.Pair<Geometry.Point2d,Geometry.Point2d>> LRTangents, Geometry.Circle agentLeft, Geometry.Circle queryRight) {
        DubinsTrajectory next = new DubinsTrajectory();
        next.type = TrajectoryType.LSR;
        double arcL1, arcL2, arcL3; //arcLengths
        Control nextControl;
        if (LRTangents.size() > 3){
            nextControl = new Control();
            //tangent pts function returns inner tangents for LR connection fourth 
            nextControl.steeringRadius = m_minTurnRadius; //left turn at max
            arcL1 = Geometry.ArcLength(agentLeft.GetPos(), m_start.pos, LRTangents.get(3).first, m_minTurnRadius, true);
            //don't use velocities because Dubins assumes unit forward velocity
            nextControl.timesteps = arcL1 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(m_start.pos.toPoint(), LRTangents.get(3).first.toPoint(), agentLeft.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = 0.0; //straight
            arcL2 = Geometry.Norm2d(LRTangents.get(3).first, LRTangents.get(3).second);
            nextControl.timesteps = arcL2 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new LineSegment(LRTangents.get(3).first.toPoint(), LRTangents.get(3).second.toPoint())); //start, end, center

            nextControl = new Control();
            nextControl.steeringRadius = -1.0 * m_minTurnRadius; //right turn at max
            arcL3 = Geometry.ArcLength(queryRight.GetPos(), LRTangents.get(3).second, m_goal.pos, m_minTurnRadius, false);
            nextControl.timesteps = arcL3 / DELTA;
            next.controls.add(nextControl);
            next.addSegment(new ArcSegment(LRTangents.get(3).second.toPoint(), m_goal.pos.toPoint(), queryRight.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center

            //calculate total length
            next.length =  arcL1 + arcL2 + arcL3;
        }
        return next;
    }




    
    private DubinsTrajectory BestCCCTrajectory(Geometry.Circle agentLeft, Geometry.Circle agentRight, Geometry.Circle queryLeft, Geometry.Circle queryRight) {
        DubinsTrajectory shortest, next;
        shortest = new DubinsTrajectory();

        //find the relative angle for L and right
        double theta  = 0.0;
        double D = Geometry.Norm2d(agentRight.GetPos(),queryRight.GetPos());

        //calculate RLR
        if (D < 4.0*m_minTurnRadius){
            theta = Math.acos(D/(4.0*m_minTurnRadius));

            theta += Math.atan2(queryRight.GetY() - agentRight.GetY(),  queryRight.GetX() - agentRight.GetX());

            next = RLRTrajectory(theta, agentRight, queryRight);
            if (next.length < shortest.length)
                shortest = next;
        }

        //calculate LRL
        D = Geometry.Norm2d(agentLeft.GetPos(),queryLeft.GetPos());
        if (D < 4.0*m_minTurnRadius){
            theta = Math.acos(D/(4.0*m_minTurnRadius));

            theta = Math.atan2(queryLeft.GetY() - agentLeft.GetY(), queryLeft.GetX() - agentLeft.GetX()) - theta;

            next = LRLTrajectory(theta, agentLeft, queryLeft);
            if (next.length < shortest.length)
                shortest = next;
        }
        return shortest;
    }

    private DubinsTrajectory RLRTrajectory(double interiorTheta, Geometry.Circle agentRight, Geometry.Circle queryRight) {
        DubinsTrajectory next = new DubinsTrajectory();
        next.type = TrajectoryType.RLR;
        double arcL1, arcL2, arcL3; //arcLengths
        Control nextControl; //for a control vector
        Geometry.Circle lCircle = new Geometry.Circle();
        lCircle.SetRadius(m_minTurnRadius);

        //compute tangent circle's pos using law of cosines + atan2 of line between agent and query circles
        lCircle.SetPos(agentRight.GetX() + (2.0*m_minTurnRadius*Math.cos(interiorTheta)), agentRight.GetY() +
                (2.0*m_minTurnRadius*Math.sin(interiorTheta)));

        //compute tangent points given tangent circle
        Geometry.Point2d agentTan = new Geometry.Point2d((lCircle.GetX() + agentRight.GetX())/2.0 , (lCircle.GetY() + agentRight.GetY())/2.0);
        Geometry.Point2d queryTan = new Geometry.Point2d((lCircle.GetX() + queryRight.GetX())/2.0 , (lCircle.GetY() + queryRight.GetY())/2.0);

        nextControl = new Control();
        nextControl.steeringRadius = -1.0 * m_minTurnRadius; //right turn at max
        arcL1 = Geometry.ArcLength(agentRight.GetPos(), m_start.pos, agentTan, m_minTurnRadius, false);
        next.addSegment(new ArcSegment(m_start.pos.toPoint(), agentTan.toPoint(), agentRight.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center

        //don't use velocities because Dubins assumes unit forward velocity
        nextControl.timesteps = arcL1 / DELTA;
        next.controls.add(nextControl);

        nextControl = new Control();
        nextControl.steeringRadius = m_minTurnRadius; //left turn at max
        arcL2 = Geometry.ArcLength(lCircle.GetPos(), agentTan, queryTan, m_minTurnRadius, true);
        nextControl.timesteps = arcL2 / DELTA;
        next.controls.add(nextControl);
        next.addSegment(new ArcSegment(agentTan.toPoint(), queryTan.toPoint(), lCircle.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center

        nextControl = new Control();
        nextControl.steeringRadius =  -1.0 * m_minTurnRadius; //right turn at max
        arcL3 = Geometry.ArcLength(queryRight.GetPos(), queryTan, m_goal.pos, m_minTurnRadius, false);
        nextControl.timesteps = arcL3 / DELTA;
        next.controls.add(nextControl);
        next.addSegment(new ArcSegment(queryTan.toPoint(), m_goal.pos.toPoint(), queryRight.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center


        //calculate total length
        next.length =  arcL1 + arcL2 + arcL3;
        return next;
    }

    private DubinsTrajectory LRLTrajectory(double interiorTheta, Geometry.Circle agentLeft, Geometry.Circle queryLeft) {
        DubinsTrajectory next = new DubinsTrajectory();
        next.type = TrajectoryType.LRL;
        double arcL1, arcL2, arcL3; //arcLengths
        Control nextControl; //for a control vector
        Geometry.Circle rCircle = new Geometry.Circle();
        rCircle.SetRadius(m_minTurnRadius);

        //compute tangent circle's pos using law of cosines + atan2 of line between agent and query circles
        rCircle.SetPos(agentLeft.GetX() + (2.0*m_minTurnRadius*Math.cos(interiorTheta)), agentLeft.GetY() + (2.0*m_minTurnRadius*Math.sin(interiorTheta)));

        //compute tangent points given tangent circle
        Geometry.Point2d agentTan = new Geometry.Point2d((rCircle.GetX() + agentLeft.GetX())/2.0 , (rCircle.GetY() + agentLeft.GetY())/2.0); 
        Geometry.Point2d queryTan = new Geometry.Point2d((rCircle.GetX() + queryLeft.GetX())/2.0 , (rCircle.GetY() + queryLeft.GetY())/2.0); 

        nextControl = new Control();
        nextControl.steeringRadius = m_minTurnRadius; //left turn at max
        arcL1 = Geometry.ArcLength(agentLeft.GetPos(), m_start.pos, agentTan, m_minTurnRadius, true);

        //don't use velocities because Dubins assumes unit forward velocity
        nextControl.timesteps = arcL1 / DELTA;
        next.controls.add(nextControl);
        next.addSegment(new ArcSegment(m_start.pos.toPoint(), agentTan.toPoint(), agentLeft.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center

        nextControl = new Control();
        nextControl.steeringRadius = -1.0 * m_minTurnRadius; //right turn at max
        arcL2 = Geometry.ArcLength(rCircle.GetPos(), agentTan, queryTan, m_minTurnRadius, false);
        nextControl.timesteps = arcL2 / DELTA;
        next.controls.add(nextControl);
        next.addSegment(new ArcSegment(agentTan.toPoint(), queryTan.toPoint(), rCircle.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.RIGHT)); //start, end, center

        nextControl = new Control();
        nextControl.steeringRadius = m_minTurnRadius; //left turn at max
        arcL3 = Geometry.ArcLength(queryLeft.GetPos(), queryTan, m_goal.pos, m_minTurnRadius, true);
        nextControl.timesteps = arcL3 / DELTA;
        next.controls.add(nextControl);
        next.addSegment(new ArcSegment(queryTan.toPoint(), m_goal.pos.toPoint(), queryLeft.GetPos().toPoint(), Constants.kMaxTargetSpeed, Constants.kMaxTargetSpeed, TURN.LEFT)); //start, end, center

        //calculate total length
        next.length =  arcL1 + arcL2 + arcL3;
        return next;
    } 
    
}