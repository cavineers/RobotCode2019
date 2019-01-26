package frc.lib;

public class FieldHelper {

    /**
     * 
     * @param targetHeadingEstimate
     */
    public static double getClosestTargetHeading(double targetHeadingEstimate) {
        targetHeadingEstimate = MathHelper.angleToNegPiToPi(targetHeadingEstimate);
        
        return 0; //TODO: implement
    }

}