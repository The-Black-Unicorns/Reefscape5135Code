// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {

  private static NetworkTable Limelight = NetworkTableInstance.getDefault().getTable("BlackUnicornsShootingLimelight");
  private static AprilTagFieldLayout fieldAprilTags;

  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private NetworkTableEntry tid;

  private final DoubleSupplier txSupplier;
  private final DoubleSupplier tySupplier;
  private final DoubleSupplier taSupplier;
  private final IntSupplier tidSupplier;


  public VisionSubsystem() {
    fieldAprilTags = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    NetworkTableEntry tx = Limelight.getEntry("tx");
    NetworkTableEntry ty = Limelight.getEntry("ty");
    NetworkTableEntry ta = Limelight.getEntry("ta");
    NetworkTableEntry tid = Limelight.getEntry("tid");

    txSupplier = () -> tx.getDouble(0.0);
    tySupplier = () -> ty.getDouble(0.0);
    taSupplier = () -> ta.getDouble(0.0);
    tidSupplier = () -> (int) tid.getDouble(-1);
  }

  private double getDistanceFromTag(){
    Optional<Pose3d> optionalTagPose = fieldAprilTags.getTagPose(tidSupplier.getAsInt());
    return optionalTagPose.isEmpty() ? -1 : 
    (optionalTagPose.get().getZ()-LIMELIGHT_HEIGHT) 
    / Math.tan((LIMELIGHT_PITCH_ANGLE + tySupplier.getAsDouble()));
  }

  private double getTxGroundPlane(){
    return Math.atan(Math.tan(txSupplier.getAsDouble())/Math.cos(LIMELIGHT_PITCH_ANGLE));
  }

  private Optional<Translation2d> getRobotPose2d(double robotYaw){
    Optional<Pose3d> optionalTagPose3d = fieldAprilTags.getTagPose(tidSupplier.getAsInt());
    double distanceFromTag = getDistanceFromTag();
    if(optionalTagPose3d.isEmpty()){
      SmartDashboard.putNumber("limelight robot x", -1);
      SmartDashboard.putNumber("limelight robot y", -1);
      return null;
    }
    
    Pose3d tagPose3d = optionalTagPose3d.get();
    double angleRelativeToTag = robotYaw + getTxGroundPlane() - Math.PI - tagPose3d.getRotation().getAngle();

    if(DriverStation.getAlliance().get() == Alliance.Red) angleRelativeToTag += Math.PI;

    angleRelativeToTag += 2*Math.PI;
    angleRelativeToTag = angleRelativeToTag % Math.PI * 2;

    Translation2d finalRobotLimelightPose = 
    new Translation2d(distanceFromTag * Math.sin(angleRelativeToTag), distanceFromTag * Math.cos(angleRelativeToTag));
    SmartDashboard.putNumber("limelight robot x", finalRobotLimelightPose.getX());
    SmartDashboard.putNumber("limelight robot y", finalRobotLimelightPose.getY());
    return Optional.of(finalRobotLimelightPose);
  }

  @Override
  public void periodic() {
  }
}
