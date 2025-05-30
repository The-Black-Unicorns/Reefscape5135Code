package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.SimulatedArena;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.optimization.SimulatedAnnealing;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArraySubscriber;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Gripper;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.GenericPresicionSystemIO.Goal;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemIO;
import frc.robot.subsystems.arm.ArmSubsystemIOSparkMax;
import frc.robot.subsystems.gripper.GripperSubsystem;
import frc.robot.subsystems.gripper.GripperSubsystemIO;
import frc.robot.subsystems.gripper.GripperSubsystemIOSim;
import frc.robot.subsystems.gripper.GripperSubsystemIOSparkMax;
import frc.robot.subsystems.gripper.GripperSubsystemIOTalonFX;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystemIO;
import frc.robot.subsystems.pivot.PivotSubsystemIOSparkMax;
import frc.robot.subsystems.swerve.SwerveSubsystemTalonFX;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystemSim;

public class SuperStructure {

    private final GripperSubsystem gripper;
    public final ArmSubsystem arm;
    public final PivotSubsystem pivot;
    private final Autonomous auto;
    
    private armStates lastpose;
    private Field2d field;
    private final StructSubscriber<Pose2d> desiredAutoPose;


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
    private boolean realRobot;
    
    public SuperStructure(){

        realRobot = Robot.isReal();


        swerve = realRobot ? 
            new SwerveSubsystem(new SwerveSubsystemTalonFX(new File(Filesystem.getDeployDirectory(), "swerve"))) :
            new SwerveSubsystem(new SwerveSubsystemSim(new File(Filesystem.getDeployDirectory(), "swerve")));

        gripper = new GripperSubsystem(
            realRobot ? new GripperSubsystemIOSparkMax(Gripper.K_SPARK_ID, Gripper.K_CURRENT_LIMIT, Gripper.K_INVERTED, Gripper.K_BRAKE)
            : new  GripperSubsystemIOSim(this)
        );

        arm = realRobot? 
            new ArmSubsystem(new ArmSubsystemIOSparkMax(Arm.LEFT_ARM_MOTOR, Arm.RIGHT_ARM_MOTOR, Arm.ARM_CURRENT_LIMIT, Arm.ARM_INVERTED, Arm.ARM_BRAKE))
            : new ArmSubsystem(new ArmSubsystemIO() {});

        // arm = new ArmSubsystem(new ArmSubsystemIOSparkMax(Arm.LEFT_ARM_MOTOR, Arm.RIGHT_ARM_MOTOR, Arm.ARM_CURRENT_LIMIT, Arm.ARM_INVERTED, Arm.ARM_BRAKE));

        
        pivot = realRobot ?
            new PivotSubsystem(new PivotSubsystemIOSparkMax(PivotConstants.PIVOT_MOTOR_ID, PivotConstants.PIVOT_CURRENT_LIMIT,
                PivotConstants.PIVOT_MOTOR_INVERTED, PivotConstants.PIVOT_MOTOR_BRAKE)) 

            : new PivotSubsystem(new PivotSubsystemIO() {});

            

        
        
        auto = new Autonomous(this);
        
        curIntakeMode = armStates.INTAKE_UP;

        desiredAutoPose = NetworkTableInstance.getDefault().getStructTopic("/PathPlanner/targetPose", Pose2d.struct)
            .subscribe(new Pose2d());

        field = new Field2d();
        SmartDashboard.putData("field", field);

        


        // new WaitCommand(0.1).andThen(() -> arm.setDefaultCommand(
        //     arm.setDesiredAngle()))
        //                     .schedule();

        arm.setDefaultCommand(arm.moveArmTargetAngle());
        pivot.setDefaultCommand(pivot.movePivotTargetAngle());

        curArmState = armStates.OUTTAKE_MIDDLE;

    }

    public Command setGoal(Goal goal){
        return Commands.parallel(
            arm.setGoal(goal),
            pivot.setGoal(goal)
        );
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
        return new RunCommand(() -> gripper.setGripperMotorSpeed(Gripper.GRIPPER_INTAKE_SPEED), gripper);
    }

    public Command outtakeCoral(){
                
        return new RunCommand(() -> gripper.setGripperMotorSpeed(Gripper.GRIPPER_OUTTAKE_SPEED), gripper);
    }

    public Command OuttakeCoralFast(){
        return new RunCommand(() -> gripper.setGripperMotorSpeed(Gripper.GRIPPER_OUTTAKEFAST_SPEED), gripper);
    }

    public Command stopGripper() {
        return new InstantCommand(() -> gripper.stopGripperMotor(), gripper);
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
            setGoal(Goal.INTAKE_DOWN),
            new InstantCommand(() -> curIntakeMode = armStates.INTAKE_DOWN)
            );
            
    }

    public Command moveArmMiddleOuttake(){
        return Commands.parallel(
            changeArmState(armStates.OUTTAKE_MIDDLE),
            setGoal(Goal.SCORE_MIDDLE)
        );
    }

    public Command moveArmUpIntake(){
        return Commands.parallel(
            changeArmState(armStates.INTAKE_UP),
            setGoal(Goal.INTAKE_UP),
            new InstantCommand(() -> curIntakeMode = armStates.INTAKE_UP)
            );

    }
    
    public Command moveArmUpOuttake(){
        return Commands.parallel(
            changeArmState(armStates.OUTTAKE_UP),
            setGoal(Goal.SCORE_UP)
        );
    }
    
    public Command moveArmToClimb(){
        return Commands.parallel(
            changeArmState(armStates.CLIMB_UP),
            setGoal(Goal.CLIMB)
        );
    }

    public Command moveArmForAuto(){
            return Commands.parallel(
                changeArmState(armStates.OUTTAKE_MIDDLE),
                arm.setTargetAngle(36),
                pivot.setTargetAngle(83));
        
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

    // public Command getAutonomousCommand() {
    //     return new WaitCommand(1).andThen(this.moveArmMiddleOuttake().andThen(
    //      Commands.sequence(
    //         new InstantCommand(() ->swerve.zeroGyroWithAlliance() , swerve),
            

    //         swerve.driveConstantSpeed(-1, 0, 0,7, true),
            

    //         // new WaitCommand(1),
    //         // this.OuttakeCoral(),
    //         // new WaitCommand(1),
    //         // this.StopGripper()

    //         this.outtakeCoral().withTimeout(1.5),
    //         this.OuttakeFast().withTimeout(0.7),
    //         this.stopGripper()
    //         // new InstantCommand(() ->swerve.zeroGyroAutonomous() , swerve)
    //     ).alongWith(arm.setDesiredAngle().alongWith(pivot.setDesiredAngle()))));

    //     // return auto.getSelected();
    // }

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

    // public Command intakeUntilCoral(){
    //     return gripper.intakeWhileNoCoral();
    // }

    // public Command OuttakeFast(){
    //     return gripper.outtakeFastCommand();
    // }

    public Command hpDropCoralSim(){
        return new InstantCommand(
            () -> swerve.hpDropCoralSimulation(), swerve);
    }


    public class AutonomousCommands{

        public Command moveArmMiddleOuttakeAuto(){
            return moveArmMiddleOuttake().andThen(
                Commands.parallel(
                    arm.moveArmTargetAngle(),
                    pivot.movePivotTargetAngle()
                ).withTimeout(0.5)
            );
        }

        public Command moveArmTopIntakeAuto(){
            return moveArmUpIntake().andThen(
                Commands.parallel(
                    arm.moveArmTargetAngle(),
                    pivot.movePivotTargetAngle()
                ).withTimeout(0.5)
            );
        }

        public Command moveArmBottomIntakeAuto(){
            return moveArmDownIntake().andThen(
                Commands.parallel(
                    arm.moveArmTargetAngle(),
                    pivot.movePivotTargetAngle()
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
            return OuttakeCoralFast();
        }
    }

    

    



    public void enabledInit(){
        arm.armEnabledInit();
        pivot.pivotEnabledInit();
        gripper.stopGripperMotor();

    }

    public void testPeriodic(){
        gripper.testPeriodic();
        arm.testPeriodic();
        pivot.testPeriodic();
    }

    public void simulationPeriodic(){
        swerve.simulationPeriodic();
        arm.simulationPeriodic();
        pivot.simulationPeriodic();
    }

    public void setIdleModeBreak(){
        arm.setIdleModeBreak();
    }
    public void setIdleModeCoast(){
        arm.setIdleModeCoast();
    }

    
    public void periodic(){

        field.getObject("Autonomous Pose").setPose(desiredAutoPose.get());
        
        field.setRobotPose(swerve.getPose());
    }

    public void autonomousInit(){
       
        if (Robot.isSimulation()){
            SimulatedArena.getInstance().resetFieldForAuto();
             gripper.gripperAutonInit();
        }
        
    }
}
