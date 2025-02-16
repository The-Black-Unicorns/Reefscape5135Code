// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static frc.robot.Constants.PivotConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotSubsystem extends SubsystemBase {
  private final SparkMax pivotMotor;
  private final SparkMaxConfig pivotMotorConfig;
  private final AbsoluteEncoder pivotAbsoluteEncoder;

  private final ArmFeedforward pivotFeedforwardController;
  private final ProfiledPIDController pivotPIDController;

  double KP, KI, KD;

  public PivotSubsystem() {

    pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD,
                    new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
    pivotPIDController.setTolerance(1);

    pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.idleMode(IdleMode.kBrake);
    // pivotMotorConfig.closedLoop.pidf(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, PIVOT_MOTOR_KF);
    pivotMotorConfig.inverted(PIVOT_MOTOR_INVERTED);
    
    pivotMotorConfig.absoluteEncoder.positionConversionFactor(360.0);
    pivotMotorConfig.absoluteEncoder.zeroOffset(PIVOT_ENCODER_OFFSET);

    pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    pivotFeedforwardController = new ArmFeedforward(PIVOT_MOTOR_KS, PIVOT_MOTOR_KG, PIVOT_MOTOR_KV, PIVOT_MOTOR_KA);
    
    pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();

    KP = PIVOT_MOTOR_KP;
    KI = PIVOT_MOTOR_KI;
    KD = PIVOT_MOTOR_KD;
  }

  private void setPosition(double targetAngleDegrees){
    double ffVoltage = pivotFeedforwardController.calculate(targetAngleDegrees* Math.PI / 180.0, 0);
    double pidVoltage = pivotPIDController.calculate(pivotAbsoluteEncoder.getPosition(), targetAngleDegrees);
    
    pivotMotor.setVoltage(ffVoltage + pidVoltage);
    
  }

  private double getPivotPosition(){
    return pivotAbsoluteEncoder.getPosition();
  }

  public Command setPivotPositionCommand(double desiredAngleDeg){
    return new RunCommand(() -> setPosition(desiredAngleDeg), this);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Pivot/pivotAngle", getPivotPosition());
  }

  public void testPeriodic(){
    double newKP = SmartDashboard.getNumber("Pivot/pivotKp", KP);
    double newKI = SmartDashboard.getNumber("Pivot/pivotKi", KI);
    double newKD = SmartDashboard.getNumber("Pivot/pivotKd", KD);
    if(newKP != KP || newKI != KI || newKD != KD){
      KP = newKP;
      KI = newKI;
      KD = newKD;
  
      pivotMotorConfig.closedLoop.pid(KP, KI, KD);
  
      pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      }
    }
}
