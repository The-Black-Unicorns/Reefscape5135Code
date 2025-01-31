// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Gripper.*;

public class GripperSubsystem extends SubsystemBase {

  // DigitalInput beamBreakSensor;
  ColorSensorV3 colorSensor;
  boolean isMotorActive;
  boolean isIntaking;

  SparkMax gripperMotor;
  SparkClosedLoopController closedLoopController;
  SparkBaseConfig configs;
  VelocityDutyCycle m_VelocityDutyCycle;
  PositionDutyCycle m_PositionDutyCycle;

  SysIdRoutine routine;
  SysIdRoutineLog logger;

  public GripperSubsystem() {
    // beamBreakSensor = new DigitalInput(K_BEAMBREAK_ID);
    colorSensor = new ColorSensorV3(Port.kOnboard);
    colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes9bit, ProximitySensorMeasurementRate.kProxRate12ms);
    isIntaking = false;
    isMotorActive = false;

    m_VelocityDutyCycle = new VelocityDutyCycle(0);
    m_PositionDutyCycle = new PositionDutyCycle(0);

    gripperMotor = new SparkMax(K_SPARK_ID, MotorType.kBrushless);
    configs = new SparkMaxConfig();
    configs.closedLoop.pid(KP, KI, KD);

    gripperMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    closedLoopController = gripperMotor.getClosedLoopController();

    MutVoltage appliedVoltage = Volts.mutable(0);
    MutAngle m_angle = Radians.mutable(0);
    MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

    routine = new SysIdRoutine(
      new Config(),
      new Mechanism(this::setMotorVoltage, log -> {
        log.motor("GripperMotor")
          .voltage(appliedVoltage.mut_replace(
            gripperMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_angle.mut_replace(gripperMotor.getAbsoluteEncoder().getPosition(), Rotations))
            .angularVelocity(m_velocity.mut_replace(gripperMotor.getAbsoluteEncoder().getVelocity()*60, RotationsPerSecond));
      },
       this));

  }


  private void intake(){
    // if(beamBreakSensor.get()){
    if(isCoral()){
      isIntaking = false;
      m_VelocityDutyCycle.Velocity = 0;
      m_PositionDutyCycle.Position = 0;
      closedLoopController.setReference(m_VelocityDutyCycle.Velocity, ControlType.kVelocity);
      isMotorActive = false;
    }
    else{
      isIntaking = true;
      m_VelocityDutyCycle.Velocity = 0.5;
      closedLoopController.setReference(m_VelocityDutyCycle.Velocity, ControlType.kVelocity);
      isMotorActive = true;
    }
  }

  public void outtake(){
    isIntaking = false;
    m_PositionDutyCycle.Position = -2;
    m_PositionDutyCycle.Velocity = 1;
      closedLoopController.setReference(m_PositionDutyCycle.Position, ControlType.kPosition);
      isMotorActive = true;
  }

  private void stopGripper(){
    isIntaking = false;
    m_VelocityDutyCycle.Velocity = 0;
    m_PositionDutyCycle.Position = 0;
    closedLoopController.setReference(m_VelocityDutyCycle.Velocity, ControlType.kVelocity);
    isMotorActive = false;
  }

  public Command intakeCommand(){
    return this.runOnce(() -> intake());
  }

  public Command stopGripperCommand(){
    return this.runOnce(() -> stopGripper());
  }

  public Command outtakeCommand() {
    
    return this.runOnce(() -> outtake());
  }
  
  public boolean isMotorRunning(){
    return isMotorActive;
  }

  public boolean isCoral(){
    // return beamBreakSensor.get();
    return colorSensor.getProximity() > 1000;
  }

  public void setMotorVoltage(Voltage v){
    gripperMotor.setVoltage(v);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  
  @Override
  public void periodic() {
    if (isCoral() && isIntaking) {
      intakeCommand().cancel();
      stopGripper();
    }

    

  }
}
