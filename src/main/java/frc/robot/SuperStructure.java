package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    private final Autonomous auto;
    public final SwerveSubsystem swerve;

    enum armStates{
        INTAKE_UP,
        OUTTAKE_UP,
        OUTTAKE_MIDDLE,
        INTAKE_DOWN
    }
    public armStates curArmState;
    public SuperStructure(){

        gripper = new GripperSubsystem();
        arm = new ArmSubsystem();
        pivot = new PivotSubsystem();
        // auto = new Autonomous();
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        
        curArmState = armStates.OUTTAKE_UP;
        auto = new Autonomous(this);


        // new WaitCommand(0.1).andThen(() -> arm.setDefaultCommand(
        //     arm.setDesiredAngle()))
        //                     .schedule();

        arm.setDefaultCommand(arm.setDesiredAngle());
        pivot.setDefaultCommand(pivot.setDesiredAngle());
    }

    // public Command ToggleGripper(){

        
    //     Command selectedMode =        
    //         Commands.either(
    //             gripper.stopGripperCommand(),
    //             Commands.either(
    //                 gripper.outtakeCommand(),
    //                 gripper.intakeCommand(),
    //             gripper::isCoral),
    //         gripper::isMotorRunning);
        
    //     System.out.println(selectedMode.getName());
    //     return selectedMode;
    // }

    public Command IntakeCoral(){
        return gripper.intakeCommand();
    }

    public Command OuttakeCoral(){
        return gripper.outtakeCommand();
    }

    public Command StopGripper() {
        return gripper.stopGripperCommand();
    }
    public Command actovateGripperCommand(){
        return Commands.either(IntakeCoral(), OuttakeCoral(),
         () -> (curArmState == armStates.INTAKE_DOWN || curArmState == armStates.INTAKE_UP));
        // return curArmState != armStates.DOWN ? IntakeCoral() :
        //     OuttakeCoral();

    }

    private Command changeArmState(armStates state){
        return Commands.runOnce(() -> changeVarArmState(state));
    }
    private void changeVarArmState(armStates wantedState){
        curArmState = wantedState;
    }
    
    public Command moveArmDownOne(){

        return Commands.either(
            moveArmUpIntake(),
            Commands.either(

                moveArmMiddleOuttake(),
                moveArmDownIntake(),
                () -> curArmState == armStates.INTAKE_UP),
             () -> curArmState == armStates.OUTTAKE_UP
        );

    }

    public Command moveArmUpOne(){
        return Commands.either(
            moveArmUpOuttake(),

            Commands.either(
            moveArmUpIntake(),
            moveArmMiddleOuttake(),
             () -> (curArmState == armStates.INTAKE_UP || curArmState == armStates.OUTTAKE_UP))
            , () -> curArmState == armStates.INTAKE_UP);
    }
    public Command moveArmDownIntake(){
        return Commands.parallel(
            changeArmState(armStates.INTAKE_DOWN),
            arm.setArmAngleDown(),
            pivot.setPivotAngleDown());
    }

    public Command moveArmMiddleOuttake(){
        return Commands.parallel(
            changeArmState(armStates.OUTTAKE_MIDDLE),
            arm.setArmAngleMiddle(),
            pivot.setPivotAngleMiddle());
    }

    public Command moveArmUpIntake(){
        return Commands.parallel(
            changeArmState(armStates.INTAKE_UP),
            arm.setArmAngleUp(),
            pivot.setPivotAngleUp()
            );
    }
    
    public Command moveArmUpOuttake(){
        return Commands.parallel(
            changeArmState(armStates.OUTTAKE_UP),
            arm.setArmAngleUp(),
            pivot.setPivotAngleUpOuttake()
        );
    }
    public Command getAutonomousCommand() {
        return Commands.sequence(
            new InstantCommand(() ->swerve.zeroGyroAutonomous() , swerve),
            Commands.parallel(
                swerve.driveConstantSpeed(0.5, 0, 0, 4, false),
                this.moveArmMiddleOuttake()
            ),
            new WaitCommand(1),
            this.OuttakeCoral(),
            new WaitCommand(1),
            this.StopGripper(),
            new InstantCommand(() ->swerve.zeroGyroAutonomous() , swerve)
        );
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

    public void periodic(){
        // if(curArmState == armStates.UP) System.out.println("bad");
        // else if(curArmState == armStates.MIDDLE) System.out.println("fine");
        // else System.out.println("greate");
    }
}
