// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.*;

import static frc.robot.subsystems.arm.ArmConstants.*;

public class Arm extends MotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {

  public enum ArmStates {
    IDLE(null),
    TOP(ARM_TOP_ANGLE),
    MID(ARM_MID_ANGLE),
    BOT(ARM_BOT_ANGLE),
    CLIMB(ARM_CLIMB_ANGLE);

    private final Double targetDeg;

    ArmStates(Double targetDeg) {
      this.targetDeg = targetDeg;
    }

    public Double position() {
      return targetDeg;
    }
  }

  private ArmStates currentState = ArmStates.IDLE;

  private final ArmFeedforward feedforwardController = 
    new ArmFeedforward(ARM_KS, ARM_KG, ARM_KV, ARM_KA);

  public Arm(
    MotorSubsystemWithFollowersConfig leadConfig,
    MotorIO leadIo,
    MotorIO[] follwerIo
  ) {
    super(leadConfig, 
      new MotorInputsAutoLogged(), 
      leadIo, new MotorInputsAutoLogged[] {new MotorInputsAutoLogged()}, 
      follwerIo);
  }

  @Override
  public void periodic() {
    super.periodic();
    stateMachine();
    telemetrize();
  }

  private void stateMachine() {
    double ffVolts = feedforwardController.calculate(
      inputs.unitPosition, inputs.velocityUnitsPerSecond);
    
    switch(currentState) {
      case IDLE -> super.setVoltageOutput(0);
      case TOP -> super.setPositionSetpoint(ArmStates.TOP.position(), ffVolts);
      case MID -> super.setPositionSetpoint(ArmStates.MID.position(), ffVolts);
      case BOT -> super.setPositionSetpoint(ArmStates.BOT.position(), ffVolts);
      default -> System.out.println("arm is really broken");
    }
  }

  private void telemetrize() {

  }

  public void setState(ArmStates newState) {
    this.currentState = newState;
  }
}
