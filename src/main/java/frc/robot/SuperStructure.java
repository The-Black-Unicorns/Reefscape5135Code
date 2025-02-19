package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SuperStructure {

    private final GripperSubsystem gripper;
    private final ArmSubsystem arm;
    private final PivotSubsystem pivot;

    // private final Autonomous auto;

    // private final Autonomous auto;
    public final SwerveSubsystem swerve;

    public SuperStructure(){

        gripper = new GripperSubsystem();
        arm = new ArmSubsystem();
        pivot = new PivotSubsystem();
        // auto = new Autonomous();
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        
        // auto = new Autonomous(this);


        // new WaitCommand(0.1).andThen(() -> arm.setDefaultCommand(
        //     arm.setDesiredAngle()))
        //                     .schedule();

        arm.setDefaultCommand(arm.setDesiredAngle());
        pivot.setDefaultCommand(pivot.setDesiredAngle());
    }

    public Command ToggleGripper(){

        
        Command selectedMode =        
            Commands.either(
                gripper.stopGripperCommand(),
                Commands.either(
                    gripper.outtakeCommand(),
                    gripper.intakeCommand(),
                gripper::isCoral),
            gripper::isMotorRunning);
        
        System.out.println(selectedMode.getName());
        return selectedMode;
    }

    // public Command setIdleModeBreak(){
    //     return new 
    // }

    public Command moveArmPlewse(DoubleSupplier speed){
        return arm.moveArmManulyCommand(speed);
    }

    public Command moveArmDown(){

        return arm.setDesiredAngleDeg(10);
    }

    public Command moveArmUp(){
        return arm.setDesiredAngleDeg(95);
         
      
    }

    public Command movePivotDown(){
        return pivot.setDesiredAngleDeg(110);
    }

    public Command movePivotUp(){
        return pivot.setDesiredAngleDeg(100);
    }

    public Command getAutonomousCommand() {
        // return auto.getSelected();
        
        return null;
    }

    public void enabledInit(){
        arm.armEnabledInit();
        pivot.pivotEnabledInit();
        gripper.stopGripper();
    }

    public void testPeriodic(){
        gripper.testPeriodic();
        arm.testPeriodic();
        pivot.testPeriodic();
    }

    public void setIdleModeBreak(){
        arm.setIdleModeBreak();
    }
    public void setIdleModeCoast(){
        arm.setIdleModeCoast();
    }
}
