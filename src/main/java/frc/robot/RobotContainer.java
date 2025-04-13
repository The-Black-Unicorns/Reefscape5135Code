// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.SuperStructure.armStates;
import frc.robot.controllers.controllers.QxDriveController;
// import frc.robot.controllers.controllers.XboxDriveController;
import frc.robot.controllers.controllers.XboxOperatorController;

public class RobotContainer {
  public SuperStructure structure;

  private QxDriveController controller;
  private XboxOperatorController operator;


  public RobotContainer() {


    controller = new QxDriveController(0);
    operator = new XboxOperatorController(1);
    

    structure = new SuperStructure();


  
    structure.swerve.setDefaultCommand(structure.swerve.driveCommandForDriver(
    controller.getXSpeed(),
    controller.getYSpeed(),
    controller.getRotationSpeed(),
    () -> true,
    controller.getSpeedPotentiometer() )
  );

  // check if works
    
    configureBindings();
  }

  private void configureBindings() {

    controller.isGripperActive().whileFalse(structure.stopGripper());
    controller.isGripperActive().whileTrue(structure.actovateGripperCommand().andThen(Commands.print("wtf")));

    controller.getIntakeMode().onFalse(structure.setDesiredState(armStates.INTAKE_UP));
    controller.getIntakeMode().onTrue(structure.setDesiredState(armStates.INTAKE_DOWN));



    operator.setArmLowAngleButton().onTrue(structure.moveArmDownIntake()
      .alongWith(structure.setDesiredState(armStates.INTAKE_DOWN))
      /* .alongWith(structure.IntakeCoral())*/);

    operator.setArmMidAngleButton().onTrue(structure.moveArmMiddleOuttake()
     /* .alongWith(structure.stopGripper())*/);

    operator.setArmTopAngleButton().onTrue(structure.moveArmUpIntake()
      .alongWith(structure.setDesiredState(armStates.INTAKE_UP))
      // .alongWith(structure.IntakeCoral())
      );
    operator.setArmClimbingAngleButton().onTrue(structure.moveArmToClimb());

    operator.intakeCoralButton().onTrue(structure.IntakeCoral());
    operator.outtakeCoralButton().onTrue(structure.outtakeCoral());
    operator.outtakeFastCoralButton().onTrue(structure.OuttakeCoralFast());

    operator.intakeCoralButton().or(operator.outtakeCoralButton().or(operator.outtakeFastCoralButton()))
      .onFalse(structure.stopGripper());

    controller.resetGyroButton().onTrue(new InstantCommand(() -> structure.swerve.zeroGyroWithAlliance()));

    if (!Robot.isReal()){
      operator.setArmLowAngleButton().onTrue(structure.hpDropCoralSim());
      operator.setArmMidAngleButton().onTrue(structure.swerve.driveToPose(new Pose2d(3.2, 4, new Rotation2d(0))));
    }

  }

  public Command getAutonomousCommand() {
    return structure.getAutonomousCommand();
  }
  
  public void periodic(){
    structure.periodic();
  }

  public void testPeriodic(){
    structure.testPeriodic();
  }
  public void enabledInit(){
    structure.enabledInit();
  }

  public void autonomousInit(){
    structure.autonomousInit();
  }
  
}
