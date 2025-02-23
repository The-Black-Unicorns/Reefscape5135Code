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

public class RobotContainer {
  private SuperStructure structure;
  private Field2d field;
  private DriveController controller;
  // private SwerveSubsystem swerve;
  
  // private DigitalInput unlockMotorsDIO;
  // private Trigger unlockMotorsTrigger;
  public RobotContainer() {
    controller = new DriveController(0);
    // autonomous = new Autonomous();
    // swerve = new SwerveSubsystem();
    field = new Field2d();
    structure = new SuperStructure();
    SmartDashboard.putData("field", field);


    // unlockMotorsDIO = new DigitalInput(0);
    // unlockMotorsTrigger = new Trigger(() -> !unlockMotorsDIO.get());

    structure.swerve.setDefaultCommand(structure.swerve.driveCommandForDriver(
    controller.getXSpeed(),
    controller.getYSpeed(),
    controller.getRotationSpeed(),
    () -> true )
  );

  // check if works
    
    configureBindings();
  }

  private void configureBindings() {
    // unlockMotorsTrigger.whileTrue(new InstantCommand(() -> structure.setIdleModeCoast()).ignoringDisable(true));
    // unlockMotorsTrigger.whileFalse(new InstantCommand(() -> structure.setIdleModeBreak()).ignoringDisable(true));
    // unlockMotorsTrigger.whileFalse(Commands.print("ahr"));
    
    // controller.intakeCoral().onTrue(structure.ToggleGripper());
    // controller.shouldArmMoveTrigger().whileTrue(structure.moveArmPlewse(
    //   () -> 0.1
    // ));
    // controller.shouldArmMoveTrigger().whileTrue(Commands.print("aa " + controller.getArmSpeed().getAsDouble()));
    controller.raiseArm().onTrue(structure.moveArmUp());
    controller.lowerArm().onTrue(structure.moveArmDown());
    controller.resetGyroButton().onTrue(new InstantCommand(() -> structure.swerve.zeroGyroForDriver()));

    //   () -> 0.5
    // ));
    // controller.shouldArmMoveTrigger().whileTrue(Commands.print("aa " + controller.getArmSpeed().getAsDouble()));
    // controller.raiseArm().onTrue(structure.movePivotUp().alongWith(structure.moveArmUp()));
    // controller.lowerArm().onTrue(structure.movePivotDown().alongWith(structure.moveArmDown()));
    controller.raiseArm().onTrue(structure.moveArmUp());
    controller.lowerArm().onTrue(structure.moveArmDown());
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
