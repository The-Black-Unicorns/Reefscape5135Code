package frc.lib.math;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveCalculations {
    
    public static Translation2d calculateForwardLimMps(Translation2d iVector, Translation2d fVector, double maxForwardAccel, double maxSpeed){
        //add edge case =0 or close to 0
        double iAngleRad = iVector.getAngle().getRadians();
        double fAngleRad = fVector.getAngle().getRadians();

        double iMag = Math.sqrt(iVector.getX() * iVector.getX() + iVector.getY() * iVector.getY());
        double fMag = Math.sqrt(fVector.getX() * fVector.getX() + fVector.getY() * fVector.getY());

        double fMagProjection = Math.cos(fAngleRad - iAngleRad) * fMag;

        double currMaxAccel = maxForwardAccel * (1-iMag/maxSpeed);

        if(Math.abs(iMag - fMagProjection) > currMaxAccel * 0.02){
            double targetMag = iMag + (fMagProjection>iMag ? (-1):1) * currMaxAccel * 0.02;
            if(fMagProjection > 0.1)
            fVector = fVector.div(fMagProjection).times(targetMag);
        }
        return fVector;
    }

    public static Translation2d calculateSkidLimMps(Translation2d iVector, Translation2d fVector, double maxSkidAccel){
        Translation2d dVel = fVector.minus(iVector);
        double dVelMag = Math.sqrt(dVel.getX() * dVel.getX() + dVel.getY() * dVel.getY());
        Rotation2d dVelAngle = dVel.getAngle();

        return (dVelMag > 0.02 * maxSkidAccel) ?
         new Translation2d(maxSkidAccel * 0.02, dVelAngle).plus(iVector) : fVector;
    }
}
