// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GripperSubsystem extends SubsystemBase {

  DigitalInput beamBreakSensor;
  BooleanSupplier isBeamBroken;
  SparkMax gripperMotor;
  boolean isMotorActive;
  public GripperSubsystem() {
    beamBreakSensor = new DigitalInput(0);

    gripperMotor = new SparkMax(0, MotorType.kBrushless);

    isMotorActive = false;
  }


  private void intake(){
    if(isBeamBroken.getAsBoolean() == true){
      gripperMotor.set(0);
      isMotorActive = false;
    }
    else{
      gripperMotor.set(0.5);
      isMotorActive = true;
    }
  }

  private void stopIntake(){
    gripperMotor.set(0);
    isMotorActive = false;
  }

  public Command intakeCommand(){
    return new RunCommand(() -> intake(), this);
  }

  public Command stopIntakeCommand(){
    return new InstantCommand(() -> stopIntake(), this);
  }
  
  public boolean isMotorRunning(){
    return isMotorActive;
  }

  @Override
  public void periodic() {
    isBeamBroken = () -> beamBreakSensor.get();
  }
}
