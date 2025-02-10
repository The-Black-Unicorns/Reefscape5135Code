// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.controllers.controllers.DriveController;
import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem;

public class RobotContainer {
  private SuperStructure structure;
  private Autonomous autonomous;
  private SwerveSubsystem swerve;
  private Field2d field;
  private DriveController controller;
    DoublePublisher xPub;
    DoublePublisher yPub;
    DoublePublisher rotPub;
  public RobotContainer() {
    controller = new DriveController(0);
    autonomous = new Autonomous();
    swerve = new SwerveSubsystem();
    field = new Field2d();
    structure = new SuperStructure();
    SmartDashboard.putData("field", field);

    swerve.setDefaultCommand(swerve.driveCommand(
    controller.getXSpeed(),
    controller.getYSpeed(),
    controller.getRotationSpeed(),
    () -> true )
  );

  // check if works
    
    configureBindings();
    

  }

  private void configureBindings() {

    controller.intakeCoral().onTrue(structure.ToggleGripper());
  }

  public Command getAutonomousCommand() {
    return autonomous.getSelected();
  }
  
  public void periodic(){
  }

  public void testPeriodic(){
    structure.testPeriodic();
  }
}
