package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SuperStructure {

    private final GripperSubsystem gripper;
    private final ArmSubsystem arm;
    private final PivotSubsystem pivot;
    private final Autonomous auto;

    public SuperStructure(){

        gripper = new GripperSubsystem();
        arm = new ArmSubsystem();
        pivot = new PivotSubsystem();
        auto = new Autonomous();
        
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
        return arm.controlArmMotor(45);
    }

    public Command moveArmUp(){
        return arm.controlArmMotor(60);
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

    public void testPeriodic(){
        gripper.testPeriodic();
        arm.testPeriodic();
        pivot.testPeriodic();
    }
}
