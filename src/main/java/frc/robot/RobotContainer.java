// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.controllers.DriveController;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem;

public class RobotContainer {
  private SuperStructure structure;
  private Field2d field;
  private DriveController controller;
  
  
  // private DigitalInput unlockMotorsDIO;
  // private Trigger unlockMotorsTrigger;
  public RobotContainer() {
    controller = new DriveController(0);
    field = new Field2d();
    structure = new SuperStructure();
    SmartDashboard.putData("field", field);


    // unlockMotorsDIO = new DigitalInput(0);
    // unlockMotorsTrigger = new Trigger(() -> !unlockMotorsDIO.get());

    structure.swerve.setDefaultCommand(structure.swerve.driveCommand(
    controller.getXSpeed(),
    controller.getYSpeed(),
    controller.getRotationSpeed(),
    () -> true )
  );
    
    configureBindings();
  }

  private void configureBindings() {
 
    controller.intakeCoral().whileTrue(structure.IntakeCoral());
    controller.outtakeCoral().whileTrue(structure.OuttakeCoral());
    controller.isGripperActive().whileFalse(structure.StopGripper());
    
  
    controller.raiseArm().onTrue(structure.moveArmUp().alongWith(structure.movePivotUp()));
    controller.lowerArm().onTrue(structure.moveArmDown().alongWith(structure.movePivotDown()));


    controller.resetGyroButton().onTrue(new InstantCommand(() -> structure.swerve.zeroHeading()));

  }

  public Command getAutonomousCommand() {
    return structure.getAutonomousCommand();
  }
  
  public void periodic(){
  }

  public void testPeriodic(){
    structure.testPeriodic();
  }
  public void enabledInit(){
    structure.enabledInit();
  }
  
}
