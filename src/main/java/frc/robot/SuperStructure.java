package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SuperStructure {

    private final GripperSubsystem gripper;
    private final ArmSubsystem arm;
    private final PivotSubsystem pivot;
    private final Autonomous auto;
    private double wantedArmAngle;

    public SuperStructure(){

        gripper = new GripperSubsystem();
        arm = new ArmSubsystem();
        pivot = new PivotSubsystem();
        auto = new Autonomous();

        wantedArmAngle = arm.getArmAngle();
        // new WaitCommand(0.1).andThen(() -> arm.setDefaultCommand(
        //     arm.setDesiredAngle()))
        //                     .schedule();

        arm.setDefaultCommand(arm.setDesiredAngle());
    }

    // public Command ToggleGripper(){

        
    //     Command selectedMode =        
    //             Commands.either(
    //                 gripper.stopGripperCommand(),
    //                 Commands.either(
    //                     gripper.outtakeCommand(),
    //                     gripper.intakeCommand(),
    //                     gripper::isCoral),
    //             gripper::isMotorRunning);
    //     System.out.println(selectedMode.getName());
    //     return selectedMode;
    // }

    // public Command setIdleModeBreak(){
    //     return new 
    // }

    public Command moveArmPlewse(DoubleSupplier speed){
        return arm.moveArmManulyCommand(speed);
    }

    public Command moveArmDown(){

        return arm.setDesiredAngleDeg(15);
    }

    public Command moveArmUp(){
        return arm.setDesiredAngleDeg(95);
         
      
    }

    public Command movePivotDown(){
        return pivot.setPivotPositionCommand(190);
    }

    public Command movePivotUp(){
        return pivot.setPivotPositionCommand(190);
    }

    public Command getAutonomousCommand() {
        return auto.getSelected();
    }

    public void enabledInit(){
        arm.armEnabledInit();
    }

    public void testPeriodic(){
        gripper.testPeriodic();
        arm.testPeriodic();
        pivot.testPeriodic();
    }
}
