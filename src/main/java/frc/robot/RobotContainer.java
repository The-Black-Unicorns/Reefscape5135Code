// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ResourceBundle.Control;

import edu.wpi.first.hal.util.UncleanStatusException;
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
  private SuperStructure structure;
  private Field2d field;
  private QxDriveController controller;
  private XboxOperatorController operator;
  // private SwerveSubsystem swerve;
  
  // private DigitalInput unlockMotorsDIO;
  // private Trigger unlockMotorsTrigger;


  private Trigger moveArmDown;
  private Trigger moveArmMid;
  private Trigger moveArmTop;

  public RobotContainer() {
    controller = new QxDriveController(0);
    operator = new XboxOperatorController(1);
    
    // autonomous = new Autonomous();
    // swerve = new SwerveSubsystem();
    field = new Field2d();
    structure = new SuperStructure();
    SmartDashboard.putData("field", field);

  
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

    // controller.raiseArmOne().onTrue(structure.moveArmMiddleOuttake());
    // controller.raiseArmOne().onTrue(structure.stopGripper());

    // controller.lowerArmOne().onTrue((structure.moveArmToPos()));
    // controller.lowerArmOne().onTrue(structure.moveArmUpIntake());
    // controller.lowerArmOne().and(structure.isArmNotMid()).onTrue(structure.IntakeCoral());

    controller.getIntakeMode().onFalse(structure.setDesiredState(armStates.INTAKE_UP));
    controller.getIntakeMode().onTrue(structure.setDesiredState(armStates.INTAKE_DOWN));



    operator.setArmLowAngleButton().onTrue(structure.moveArmDownIntake().alongWith(structure.setDesiredState(armStates.INTAKE_DOWN)));
    operator.setArmMidAngleButton().onTrue(structure.moveArmMiddleOuttake());
    operator.setArmTopAngleButton().onTrue(structure.moveArmUpIntake().alongWith(structure.setDesiredState(armStates.INTAKE_UP)));

    operator.intakeCoralButton().onTrue(structure.intakeUntilCoral());
    operator.outtakeCoralButton().onTrue(structure.outtakeCoral());

    operator.intakeCoralButton().or(operator.outtakeCoralButton()).onFalse(structure.stopGripper());
  
    // controller.getIntakeMode().onTrue(structure.moveArmDownIntake());

    controller.resetGyroButton().onTrue(new InstantCommand(() -> structure.swerve.zeroGyroWithAlliance()));

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
  
}
