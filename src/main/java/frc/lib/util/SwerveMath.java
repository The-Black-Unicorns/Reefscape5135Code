package frc.lib.util;

import static frc.robot.Constants.swerveMathConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveMath {
    public static Translation2d normalizeWheelAccel(Translation2d curTranslationSpeeds, Translation2d lastTranslationSpeeds){
        Translation2d normalizedSpeeds;

        Translation2d forwardLimit = forwardAccelLimit(curTranslationSpeeds, lastTranslationSpeeds);
        Translation2d skidLimit = skidAccelLimit(curTranslationSpeeds, lastTranslationSpeeds);
        
        normalizedSpeeds = new Translation2d(Math.min(forwardLimit.getX(), skidLimit.getX()), Math.min(forwardLimit.getY(), skidLimit.getY()));
        return normalizedSpeeds;
    }

    private static Translation2d forwardAccelLimit(Translation2d curSpeeds, Translation2d lastSpeeds){
        Rotation2d forwardAngle = lastSpeeds.getAngle();
        Rotation2d curAngle = curSpeeds.getAngle();
        Rotation2d angleDiff = forwardAngle.minus(curAngle);

        Translation2d forwardProjection = new Translation2d(angleDiff.getCos() * Math.hypot(curSpeeds.getX(), curSpeeds.getY()), forwardAngle);
        Translation2d remainder = curSpeeds.minus(forwardProjection);
        
        double forwardDiff = Math.hypot(forwardProjection.getX(), forwardProjection.getY()) - Math.hypot(lastSpeeds.getX(), lastSpeeds.getY());
        if(Math.abs(forwardDiff) > MAX_FORWARD_ACCEL * ROBOT_CYCLE){
            Translation2d res;
            if(forwardDiff < 0){
                double size = Math.hypot(lastSpeeds.getX(), lastSpeeds.getY()) - MAX_FORWARD_ACCEL * ROBOT_CYCLE;
                res  = (new Translation2d(size, forwardAngle)).plus(remainder);
                return res;
            } else if(forwardDiff>0){
                double size = Math.hypot(lastSpeeds.getX(), lastSpeeds.getY()) + MAX_FORWARD_ACCEL * ROBOT_CYCLE;
                res  = (new Translation2d(size, forwardAngle)).plus(remainder);
                return res;
            }
        }
        return curSpeeds;
    }

    private static Translation2d skidAccelLimit(Translation2d curSpeeds, Translation2d lastSpeeds){
        Translation2d difference = curSpeeds.minus(lastSpeeds);
        double diffSize = Math.hypot(difference.getX(), difference.getY());


        if(diffSize == 0){
            return new Translation2d();
        }
        if(diffSize > MAX_SKID_ACCEL * ROBOT_CYCLE){
            Translation2d fSpeeds;
            fSpeeds = new Translation2d(MAX_SKID_ACCEL*ROBOT_CYCLE, difference.getAngle());
            return lastSpeeds.plus(fSpeeds);
        }
        return curSpeeds;
    }
}
