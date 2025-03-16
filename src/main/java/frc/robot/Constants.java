package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public class Constants {

    public static final class swerveMathConstants {
        public static final double MAX_SKID_ACCEL = 100; // mps2
        public static final double MAX_FORWARD_ACCEL = 5; // mps2
        public static final double MAX_ROBOT_SIDES_ACCEL = 100; // mps2

        public static final double ROBOT_CYCLE = 0.02; // s
    }
    public static final class ControllerConstants {
        public static final double STICK_DEADBAND = 0.04;
        public static final double BACK_BUTTONS_DEADBAND = 0.15;
    }

    public static final class PivotConstants{ //change everything!!!
        public static final int PIVOT_MOTOR_ID = 19;

        public static final double PIVOT_MOTOR_KP = 0.16;
        public static final double PIVOT_MOTOR_KI = 0;
        public static final double PIVOT_MOTOR_KD = 0;
        public static final double PIVOT_MOTOR_KF = 1;

        public static final double PIVOT_MOTOR_KS = 0.01;
        public static final double PIVOT_MOTOR_KV = 0.01;
        public static final double PIVOT_MOTOR_KG = 0.04;
        public static final double PIVOT_MOTOR_KA = 0;

        public static final boolean PIVOT_MOTOR_INVERTED = false;

        public static final double PIVOT_ENCODER_OFFSET = 207.2-1.5;
        public static final double PIVOT_NORMALIZE_OFFSET = 0;
        public static final double POSITION_CONVERSION_FACTOR = 360;

        public static final double MAX_PIVOT_DEGREES_PER_SECOND = 150;
        public static final double MAX_PIVOT_DEGREES_PER_SECOND_SQUARED = 550;

        public static final double PIVOT_POSITION_TOLERANCE_DEG = 1.0;


        public static final double PIVOT_TOP_OUTTAKE_ANGLE = 213.0;
        public static final double PIVOT_TOP_ANGLE = 185; //185
        public static final double PIVOT_MID_ANGLE = 92.0; //92
        public static final double PIVOT_BOT_ANGLE = 97.5;

    }
    
    public static final class Gripper {
        public static final double GRIPPER_KP = 0.1;
        public static final double GRIPPER_KI = 0;
        public static final double GRIPPER_KD = 0;
        public static final double KMAX_ACCEL = 0.5;
        public static final double KMAX_SPEED = 1;

        public static final int K_SPARK_ID = 18; 
        public static final int K_BEAMBREAK_ID = 1;
    }

        /* Arm constants */
    /* FF constants calculated from Recalc - not final */
    public class Arm{

        public static final double ARM_MAX_VELOCITY = 800, ARM_MAX_ACCELARATION = 1000; // deg/s, deg/s2
        // public static final double ARM_KS = 0.1,ARM_KV = 2.94,ARM_KA = 0.01,ARM_KG = 0.57; 

        public static final double ARM_KS = 0.01;
        public static final double ARM_KV = 0.1;
        public static final double ARM_KA = 0.0;
        public static final double ARM_KG = 0.1; //0.32

        public static final double ARM_KP = 0.26;  // previus was 0.13
        public static final double ARM_KI = 0; // previus was 0.0
        public static final double ARM_KD = 0.01;  // previus was 0.02

        public static final int ARM_CURRENT_LIMIT = 12;
        public static final double ARM_ENCODER_OFFSET = 55.0-16 - 40+74;
        public static final double ARM_NORMALIZE_OFFSET = 0;
        public static final double ARM_POSITION_TOLERANCE_DEG = 1.0;
        
        public static final int RIGHT_ARM_MOTOR = 9;
        public static final int LEFT_ARM_MOTOR = 8;

        public static final double ARM_MID_ANGLE = 47.5; //47.5
        public static final double ARM_TOP_ANGLE = 85;  //76
        public static final double ARM_BOT_ANGLE = 355.5; //355.5
        public static final double ARM_CLIMB_ANGLE = 345;



        // public static final TrapezoidProfile.Constraints ANGLE_CONSTRAINTS = new TrapezoidProfile.Constraints(360, 720);
    }

    public static final class Swerve {

        public static final String LIMELIGHT_NAME = "limelight-tsachi";
        // public static final double MAX_FORWARD_ACCEL = 5; // MPS^2 
        // public static final double MAX_SKID_ACCEL = 5; //MPS^2

        public static final COTSTalonFXSwerveConstants CHOOSEN_MODULE = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.SDS.MK4N.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4N.driveRatios.L2PLUS);

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = 0.55;
         // TODO: This must be tuned to specific robot
        public static final double WHEELBASE = 0.55; // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = CHOOSEN_MODULE.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0));

        /* Module Gear Ratios */
        public static final double DRIVE_GEAR_RATIO = CHOOSEN_MODULE.driveGearRatio;
        public static final double ANGLE_GEAR_RATIO = CHOOSEN_MODULE.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue ANGLE_MOTOR_INVERT = CHOOSEN_MODULE.angleMotorInvert;
        public static final InvertedValue DRIVE_MOTOR_INVERT = CHOOSEN_MODULE.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = CHOOSEN_MODULE.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int ANGLE_CURRENT_LIMIT = 25;
        public static final int ANGLE_CURRENT_THRESHOLD = 40;
        public static final double ANGLE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CURRENT_LIMIT = 35;
        public static final int DRIVE_CURRENT_THRESHOLD = 60;
        public static final double DRIVE_CURRENT_THRESHOLD_TIME = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double OPEN_LOOP_RAMP = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 40;
        public static final double ANGLE_KI = 0;
        public static final double ANGLE_KD = 0
        ;

        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.12; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double DRIVE_KS = 0.32; // TODO: This must be tuned to specific robot
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
         public static final double MAX_SPEED = 5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double MAX_ANGULAR_VELOCITY = 9.5; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 11;
            public static final int ANGLE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 13;

            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(140);
           public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 21;
            public static final int ANGLE_MOTOR_ID = 22;
            public static final int CANCODER_ID = 23;

            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(-50);
           public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 31;
            public static final int ANGLE_MOTOR_ID = 32;
            public static final int CANCODER_ID = 33;

            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(105);
           public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int DRIVE_MOTOR_ID = 41;
            public static final int ANGLE_MOTOR_ID = 42;
            public static final int CANCODER_ID = 43;

            // public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(134-2.64);
        //     public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(131.57-182.4);
        //    public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
        }


        
    }
    




       
}

