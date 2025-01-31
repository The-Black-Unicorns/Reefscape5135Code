package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.*;

public class RobotState {
    private static AHRS navxAhrs;
    private Translation3d robotPosition; //matrix {x, y, theta}
    private Translation3d robotPositionError;

    private Translation3d navxRobotPosition; //{x,y theta}, x and y are not reliable, big error
    private Translation3d navxRobotPositionError;

    private Translation2d oddometryRobotPosition; //{x, y}
    private Translation2d oddometryRobotPositionError;
    
    private Translation3d visionRobotPosition; // {x, y, theta}, reliable
    private Translation3d visionRobotPositionError;

    public RobotState(){
        navxAhrs = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);
    }
    
    public static Translation2d getAccel(){
        return new Translation2d(navxAhrs.getRawAccelX(), navxAhrs.getRawAccelY());
    }



}
