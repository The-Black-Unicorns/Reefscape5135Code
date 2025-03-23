// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Gripper.*;

public class GripperSubsystem extends SubsystemBase {


  // DigitalInput beamBreakSensor;
  // ColorSensorV3 colorSensor;
  boolean isMotorActive;
  boolean isIntaking;

  private SparkMax gripperMotor;
  private SparkClosedLoopController closedLoopController;
  private SparkBaseConfig configs;
  private VelocityDutyCycle m_VelocityDutyCycle;
  private PositionDutyCycle m_PositionDutyCycle;

  private double KP, KI, KD;

  SysIdRoutine routine;
  SysIdRoutineLog logger;

  public GripperSubsystem() {
    // beamBreakSensor = new DigitalInput(K_BEAMBREAK_ID);
    // colorSensor = new ColorSensorV3(Port.kOnboard);
    // colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes9bit, ProximitySensorMeasurementRate.kProxRate12ms);
    isIntaking = false;
    isMotorActive = false;

    m_VelocityDutyCycle = new VelocityDutyCycle(0);
    m_PositionDutyCycle = new PositionDutyCycle(0);

    gripperMotor = new SparkMax(K_SPARK_ID, MotorType.kBrushless);
    configs = new SparkMaxConfig();
    configs.idleMode(IdleMode.kBrake);
    configs.closedLoop.pid(GRIPPER_KP, GRIPPER_KI, GRIPPER_KD);
    configs.inverted(true);

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

       KP = GRIPPER_KP;
       KI = GRIPPER_KI;
       KD = GRIPPER_KD;
  }


  private void intake(){

    if(isCoral()){
      isIntaking = false;
      m_VelocityDutyCycle.Velocity = 0;
      m_PositionDutyCycle.Position = 0;
      gripperMotor.set(0);
      isMotorActive = false;
    }
    else{
      isIntaking = true;
      m_VelocityDutyCycle.Velocity = 0.1;
      
      gripperMotor.set(0.85);
      isMotorActive = true;
    }
  }

  public void outtake(){
    isIntaking = false;
    m_PositionDutyCycle.Position = -2;
    m_PositionDutyCycle.Velocity = 1;
    gripperMotor.set(-0.24);
      isMotorActive = true;
  }

  public void outtakeFast(){
    isIntaking = false;
    m_PositionDutyCycle.Position = -2;
    m_PositionDutyCycle.Velocity = 1;
    gripperMotor.set(-0.37);
    isMotorActive = true;
  }

  public void stopGripper(){
    isIntaking = false;
    m_VelocityDutyCycle.Velocity = 0;
    m_PositionDutyCycle.Position = 0;
    gripperMotor.set(0);
    isMotorActive = false;
  }

  public Command intakeCommand(){
    return this.run(() -> intake());
  }

  public Command stopGripperCommand(){
    return Commands.sequence(
      this.runOnce(() -> stopGripper()),
      this.runOnce(() -> intakeWhileNoCoral().cancel()),
      this.runOnce(() -> intakeCommand().cancel()) 
    );
  }

  public Command outtakeCommand() {
    
    return this.run(() -> outtake());
  }

  public Command outtakeFastCommand() {
    
    return this.run(() -> outtakeFast());
  }

  public Command intakeWhileNoCoral(){

    return this.run(() -> intake()).until(this::isCoral);
  }
  
  public boolean isMotorRunning(){
    return isMotorActive;
  }

  public boolean isCoral(){

    // return colorSensor.getProximity() > 1000;
    return false;
  }

  public boolean isNotCoral(){
    return !isCoral();

  }

  private void setMotorVoltage(Voltage v){
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
    SmartDashboard.putBoolean("Gripper/isCoral", SmartDashboard.getBoolean("Gripper/isCoral", false));
  }

public void testPeriodic(){
  SmartDashboard.putNumber("Gripper/gripperKp", SmartDashboard.getNumber("Gripper/gripperKp", 0));
  SmartDashboard.putNumber("Gripper/gripperKi", SmartDashboard.getNumber("Gripper/gripperKi", 0));
  SmartDashboard.putNumber("Gripper/gripperKd", SmartDashboard.getNumber("Gripper/gripperKd", 0));
  double newKP = SmartDashboard.getNumber("Gripper/gripperKp", KP);
  double newKI = SmartDashboard.getNumber("Gripper/gripperKi", KI);
  double newKD = SmartDashboard.getNumber("Gripper/gripperKd", KD);
  if(newKP != KP || newKI != KI || newKD != KD){
    KP = newKP;
    KI = newKI;
    KD = newKD;

    configs.closedLoop.pid(KP, KI, KD);

    gripperMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }


}
