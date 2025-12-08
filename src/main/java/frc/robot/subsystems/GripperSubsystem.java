// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Gripper.*;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.MotorSubsystem;

public class GripperSubsystem extends MotorSubsystem<MotorInputsAutoLogged, MotorIO> {

  boolean isMotorActive;
  boolean isIntaking;

  private VelocityDutyCycle m_VelocityDutyCycle;
  private PositionDutyCycle m_PositionDutyCycle;

  private double KP, KI, KD;

  SysIdRoutine routine;
  SysIdRoutineLog logger;

  public GripperSubsystem(MotorIO io) {
    // beamBreakSensor = new DigitalInput(K_BEAMBREAK_ID);
    // colorSensor = new ColorSensorV3(Port.kOnboardCS3);
    // colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes9bit,
    // ProximitySensorMeasurementRate.kProxRate12ms);
    super(new MotorInputsAutoLogged(), io, gripperConfig);
    isIntaking = false;
    isMotorActive = false;

    m_VelocityDutyCycle = new VelocityDutyCycle(0);
    m_PositionDutyCycle = new PositionDutyCycle(0);
  }

  private void intake() {
    super.setVoltageOutput(0.85);
    // if (isCoral()) {
    //   isIntaking = false;
    //   m_VelocityDutyCycle.Velocity = 0;
    //   m_PositionDutyCycle.Position = 0;
    //   super.setVoltageOutput(0);
    //   ;
    //   isMotorActive = false;
    // } else {
    //   isIntaking = true;
    //   m_VelocityDutyCycle.Velocity = 0.1;

    //   super.setVoltageOutput(0.85);
    //   isMotorActive = true;
    // }
  }

  public void outtake() {
    super.setVoltageOutput(-0.24);
    // isIntaking = false;
    // m_PositionDutyCycle.Position = -2;
    // m_PositionDutyCycle.Velocity = 1;
    // isMotorActive = true;
  }

  public void outtakeFast() {
    super.setVoltageOutput(-0.37 * 12);
    // isIntaking = false;
    // m_PositionDutyCycle.Position = -2;
    // m_PositionDutyCycle.Velocity = 1;
    // isMotorActive = true;
  }

  public void stopGripper() {
    super.setVoltageOutput(0);
    // isIntaking = false;
    // m_VelocityDutyCycle.Velocity = 0;
    // m_PositionDutyCycle.Position = 0;
    // isMotorActive = false;
  }

  public Command intakeCommand() {
    return this.run(() -> intake());
  }

  public Command stopGripperCommand() {
    return Commands.sequence(
        this.runOnce(() -> stopGripper()),
        this.runOnce(() -> intakeWhileNoCoral().cancel()),
        this.runOnce(() -> intakeCommand().cancel()));
  }

  public Command outtakeCommand() {

    return this.run(() -> outtake());
  }

  public Command outtakeFastCommand() {

    return this.run(() -> outtakeFast());
  }

  public Command intakeWhileNoCoral() {

    return this.run(() -> intake()).until(this::isCoral);
  }

  public boolean isMotorRunning() {
    return isMotorActive;
  }

  public boolean isCoral() {

    // return colorSensor.getProximity() > 1000;
    return false;
  }

  public boolean isNotCoral() {
    return !isCoral();
  }

  @Override
  public void periodic() {
    super.periodic();
    if (isCoral() && isIntaking) {
      intakeCommand().cancel();
      stopGripper();
    }
    SmartDashboard.putBoolean("Gripper/isCoral", this.isCoral());
  }

  public void testPeriodic() {
    // SmartDashboard.putNumber("Gripper/gripperKp", SmartDashboard.getNumber("Gripper/gripperKp",
    // 0));
    // SmartDashboard.putNumber("Gripper/gripperKi", SmartDashboard.getNumber("Gripper/gripperKi",
    // 0));
    // SmartDashboard.putNumber("Gripper/gripperKd", SmartDashboard.getNumber("Gripper/gripperKd",
    // 0));
    // double newKP = SmartDashboard.getNumber("Gripper/gripperKp", KP);
    // double newKI = SmartDashboard.getNumber("Gripper/gripperKi", KI);
    // double newKD = SmartDashboard.getNumber("Gripper/gripperKd", KD);
    // if(newKP != KP || newKI != KI || newKD != KD){
    //   KP = newKP;
    //   KI = newKI;
    //   KD = newKD;

    //   configs.closedLoop.pid(KP, KI, KD);

    //   gripperMotor.configure(configs, ResetMode.kResetSafeParameters,
    // PersistMode.kNoPersistParameters);
  }
}
