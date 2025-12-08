package frc.robot.subsystems.arm;

import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.subsystems.MotorSubsystemWithFollowersConfig;

public class ArmConstants {
    public static final double ARM_MAX_VELOCITY = 1000, ARM_MAX_ACCELARATION = 800; // deg/s, deg/s2
    // public static final double ARM_KS = 0.1,ARM_KV = 2.94,ARM_KA = 0.01,ARM_KG = 0.57;

    public static final double ARM_KS = 0.01;
    public static final double ARM_KV = 0.1;
    public static final double ARM_KA = 0.0;
    public static final double ARM_KG = 0.1; // 0.32

    // public static final double ARM_KP = 0.25; // previus was 0.13
    // public static final double ARM_KI = 0; // previus was 0.0
    // public static final double ARM_KD = 0.03; // previus was 0.02

    public static final int ARM_CURRENT_LIMIT = 40;
    public static final double ARM_ENCODER_OFFSET = 205;
    public static final double ARM_NORMALIZE_OFFSET = 0;
    public static final double ARM_POSITION_TOLERANCE_DEG = 1.0;

    public static final int RIGHT_ARM_MOTOR = 9;
    public static final int LEFT_ARM_MOTOR = 8;

    public static final double ARM_MID_ANGLE = 44.5; // 47.5
    public static final double ARM_TOP_ANGLE = 53.7; // 76
    public static final double ARM_BOT_ANGLE = 351.5; // 351.5
    public static final double ARM_CLIMB_ANGLE = 345;

    public static MotorSubsystemWithFollowersConfig.FollowerConfig
        followerLeftConfig = new MotorSubsystemWithFollowersConfig.FollowerConfig();
    public static MotorSubsystemWithFollowersConfig ArmConfig = new MotorSubsystemWithFollowersConfig(); 
    static {
        ArmConfig.name = "Arm";
        ArmConfig.id = 9;
        ArmConfig.sparkConfig = new SparkMaxConfig();

        ArmConfig.sparkConfig.inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ARM_CURRENT_LIMIT)
            .closedLoop.maxMotion.allowedClosedLoopError(ARM_POSITION_TOLERANCE_DEG)
                .maxAcceleration(ARM_MAX_ACCELARATION)
                .maxVelocity(ARM_MAX_VELOCITY);

        ArmConfig.sparkConfig.closedLoop.pid(0.01, 0, 0);
                
        ArmConfig.sparkConfig.absoluteEncoder.zeroOffset(ARM_ENCODER_OFFSET);

        ArmConfig.unitToRotorRatio = 7.0/3.0;
        ArmConfig.absoluteEncoderToRotorRatio = 7.0/3.0;

        ArmConfig.kMinPositionUnits = -30.0;
        ArmConfig.kMaxPositionUnits = 92.0;
        ArmConfig.usingAbsoluteEncoder = true;
        ArmConfig.momentOfInertia = 2.0;

        followerLeftConfig.config = ArmConfig;
        followerLeftConfig.config.name = "ArmLeftMotor";
        followerLeftConfig.config.id = 8;
        followerLeftConfig.config.usingAbsoluteEncoder = false;
        followerLeftConfig.inverted = false;

        ArmConfig.followerConfigs = new MotorSubsystemWithFollowersConfig.FollowerConfig[] 
            {followerLeftConfig};
    }
}
