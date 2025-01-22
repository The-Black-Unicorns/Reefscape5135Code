package frc.lib.math;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveCalculations {
    
    public static Translation2d calculateForwardLim(double vxi, double vyi, double vxf, double vyf, double maxForwardAccel, double maxSpeed){
        Translation2d iVector = new Translation2d(vxi, vyi);
        Translation2d fVector = new Translation2d(vxf, vyf);

        double iAngleRad = iVector.getAngle().getRadians();
        double fAngleRad = fVector.getAngle().getRadians();

        double iMag = Math.sqrt(vxi*vxi + vyi*vyi);
        double fMag = Math.sqrt(vxf*vxf + vyf*vyf);

        double fMagProjection = Math.cos(fAngleRad - iAngleRad) * fMag;

        double currMaxAccel = maxForwardAccel * (1-iMag/maxSpeed);

        if(Math.abs(iMag - fMagProjection) < currMaxAccel * 0.02){
            return new Translation2d(vxf, vyf);
        }
        else{
            double targetMag = iMag + (fMagProjection>iMag ? (-1):1) * currMaxAccel * 0.02;
            if(iMag > 0.1)
            fVector = fVector.div(iMag).times(targetMag);
            return fVector;
        }
    }
}
