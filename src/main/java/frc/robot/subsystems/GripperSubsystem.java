// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Gripper.*;

public class GripperSubsystem extends SubsystemBase {

  DigitalInput beamBreakSensor;
  BooleanSupplier isBeamBroken;
  boolean isMotorActive;

  SparkMax gripperMotor;
  SparkClosedLoopController closedLoopController;
  SparkBaseConfig configs;
  VelocityDutyCycle m_VelocityDutyCycle;
  PositionDutyCycle m_PositionDutyCycle;

  public GripperSubsystem() {
    beamBreakSensor = new DigitalInput(K_BEAMBREAK_ID);
    isMotorActive = false;
    
    m_VelocityDutyCycle = new VelocityDutyCycle(0);
    m_PositionDutyCycle = new PositionDutyCycle(0);

    gripperMotor = new SparkMax(K_SPARK_ID, MotorType.kBrushless);
    configs = new SparkMaxConfig();
    configs.closedLoop.pid(KP, KI, KD);

    gripperMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    closedLoopController = gripperMotor.getClosedLoopController();


  }


  private void intake(){
    if(isBeamBroken.getAsBoolean() == true){
      m_VelocityDutyCycle.Velocity = 0;
      closedLoopController.setReference(m_VelocityDutyCycle.Velocity, ControlType.kVelocity);
      isMotorActive = false;
    }
    else{
      m_VelocityDutyCycle.Velocity = 0.5;
      closedLoopController.setReference(m_VelocityDutyCycle.Velocity, ControlType.kVelocity);
      isMotorActive = true;
    }
  }

  public void outtake(){
    m_PositionDutyCycle.Position = -1;
    m_PositionDutyCycle.Velocity = 1;
      closedLoopController.setReference(m_PositionDutyCycle.Position, ControlType.kPosition);
      isMotorActive = true;
  }

  private void stopIntake(){
    closedLoopController.setReference(0, ControlType.kVelocity);
    isMotorActive = false;
  }

  public Command intakeCommand(){
    return new RunCommand(() -> intake(), this);
  }

  public Command stopIntakeCommand(){
    return new InstantCommand(() -> stopIntake(), this);
  }

  public Command outtakeCommand() {
    return new RunCommand(() -> stopIntake(), this);
  }
  
  public boolean isMotorRunning(){
    return isMotorActive;
  }

  public boolean isCoral(){
    return isBeamBroken.getAsBoolean();
  }

  @Override
  public void periodic() {
    isBeamBroken = () -> beamBreakSensor.get();
  }
}
