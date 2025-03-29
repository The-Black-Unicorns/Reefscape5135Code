package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SuperStructure {

    private final GripperSubsystem gripper;
    public final ArmSubsystem arm;
    public final PivotSubsystem pivot;
    private final Autonomous auto;
    
    private armStates lastpose;


    // private final Autonomous auto;

    // private final Autonomous auto;
    public final SwerveSubsystem swerve;

    enum armStates{
        INTAKE_UP,
        OUTTAKE_UP,
        OUTTAKE_MIDDLE,
        INTAKE_DOWN,
        CLIMB_UP,
        CLIMB_DOWN
    }

    public armStates curArmState;
    private armStates curIntakeMode;
    
    public SuperStructure(){

        gripper = new GripperSubsystem();
        arm = new ArmSubsystem();
        pivot = new PivotSubsystem();
        auto = new Autonomous(this);
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        
        
        curIntakeMode = armStates.INTAKE_UP;

        


        // new WaitCommand(0.1).andThen(() -> arm.setDefaultCommand(
        //     arm.setDesiredAngle()))
        //                     .schedule();

        arm.setDefaultCommand(arm.setDesiredAngle());
        pivot.setDefaultCommand(pivot.setDesiredAngle());

        curArmState = armStates.OUTTAKE_MIDDLE;

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

    public Command outtakeCoral(){
        return gripper.outtakeCommand();
    }

    public Command stopGripper() {
        return gripper.stopGripperCommand();
    }
    public Command actovateGripperCommand(){
        return Commands.either(IntakeCoral(), outtakeCoral(),
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
            pivot.setPivotAngleDown(),
            new InstantCommand(() -> curIntakeMode = armStates.INTAKE_DOWN)
            );
            
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
            pivot.setPivotAngleUp(),
            new InstantCommand(() -> curIntakeMode = armStates.INTAKE_UP)
            );

    }
    
    public Command moveArmUpOuttake(){
        return Commands.parallel(
            changeArmState(armStates.OUTTAKE_UP),
            arm.setArmAngleUp(),
            pivot.setPivotAngleUpOuttake()
        );
    }
    
    public Command moveArmToClimb(){
        return Commands.parallel(
            changeArmState(armStates.CLIMB_UP),
            arm.setArmAngleToClimb(),
            pivot.setPivotAngleUpOuttake()
        );
    }

    public Command moveArmForAuto(){
            return Commands.parallel(
                changeArmState(armStates.OUTTAKE_MIDDLE),
                arm.setDesiredAngleDeg(36),
                pivot.setDesiredAngleDeg(83));
        
    }
    public Command getAutonomousCommand() {
        // return new WaitCommand(1).andThen(this.moveArmMiddleOuttake().andThen(
        //  Commands.sequence(
        //     new InstantCommand(() ->swerve.zeroGyroWithAlliance() , swerve),
            

        //     swerve.driveConstantSpeed(-1, 0, 0,7, true),
            

        //     // new WaitCommand(1),
        //     // this.OuttakeCoral(),
        //     // new WaitCommand(1),
        //     // this.StopGripper()

        //     this.outtakeCoral().withTimeout(1),
        //     this.OuttakeFast().withTimeout(1),
        //     this.stopGripper()
        //     // new InstantCommand(() ->swerve.zeroGyroAutonomous() , swerve)
        // ).alongWith(arm.setDesiredAngle().alongWith(pivot.setDesiredAngle()))));

        return auto.getSelected();
    }

    public Command moveArmToPos(){
        return Commands.either(
            moveArmUpIntake(), Commands.either(
                moveArmDownIntake(),
                Commands.print("nuh uh")
                 , () -> curIntakeMode == armStates.INTAKE_DOWN),
        () -> curIntakeMode == armStates.INTAKE_UP);
    }

    public Command armSourceScoreToggle(){
        return Commands.either(
            Commands.sequence(
                moveArmUpIntake(),
                IntakeCoral()), moveArmMiddleOuttake(), () -> curArmState == armStates.OUTTAKE_MIDDLE);
        
    }

    public Command armGroundScoreToggle(){
        return Commands.either(
            Commands.sequence(
                moveArmDownIntake(),
                IntakeCoral()
            ), moveArmMiddleOuttake(), () -> curArmState == armStates.OUTTAKE_MIDDLE);
    }

    public Command setArmLastState(){
       return new InstantCommand(() ->  curArmState = curIntakeMode);
    }

    public Command setDesiredState(armStates state){
        return new InstantCommand(() -> curIntakeMode = state);
    }

    public BooleanSupplier isArmNotMid(){
        return () -> !(curArmState == armStates.OUTTAKE_MIDDLE);
    }

    public Command intakeUntilCoral(){
        return gripper.intakeWhileNoCoral();
    }

    public Command OuttakeFast(){
        return gripper.outtakeFastCommand();
    }


    public class AutonomousCommands{

        public Command moveArmMiddleOuttakeAuto(){
            return moveArmMiddleOuttake().andThen(
                Commands.parallel(
                    arm.setDesiredAngle(),
                    pivot.setDesiredAngle()
                ).withTimeout(0.5)
            );
        }

        public Command moveArmTopIntakeAuto(){
            return moveArmUpIntake().andThen(
                Commands.parallel(
                    arm.setDesiredAngle(),
                    pivot.setDesiredAngle()
                ).withTimeout(0.5)
            );
        }

        public Command moveArmBottomIntakeAuto(){
            return moveArmDownIntake().andThen(
                Commands.parallel(
                    arm.setDesiredAngle(),
                    pivot.setDesiredAngle()
                ).withTimeout(0.5)
            );
        }

        public Command stopGripperAuto(){
            return stopGripper();
        }

        public Command intakeCoralAuto(){
            return IntakeCoral();
        }

        public Command outtakeCoralAuto(){
            return outtakeCoral();
        }

        public Command outtakeFastAuto(){
            return OuttakeFast();
        }
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
        // SmartDashboard.putString("armState", curArmState.name());
    }
}
