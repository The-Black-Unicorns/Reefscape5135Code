package frc.robot;

import com.fasterxml.jackson.databind.introspect.AnnotatedMethod;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SuperStructure.AutonomousCommands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystemTalonFX;

import static frc.robot.Constants.AutoConstants;
public class Autonomous {
    private final SwerveSubsystem drive;
    private final AutonomousCommands autoStructure;
    private final SendableChooser<Command> autoChooser;
    private AutoBuilder builder;
    private RobotConfig config;
    

    public Autonomous(SuperStructure structure){
        drive = structure.swerve;
        // autoChooser = new AutoChooser();
        autoStructure = structure.new AutonomousCommands();

        
        

        // autoFactory = new AutoFactory(
        //     drive::getPose,
        //     drive::setPose,
        //     drive::followTrajectory,
        //     false,
        //     drive
        // );

        
        builder = new AutoBuilder();
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e){
            e.printStackTrace();
        }

        AutoBuilder.configure(
            drive::getPose, // Robot pose supplier
            drive::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            drive::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> drive.drive(new Translation2d(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond),speeds.omegaRadiansPerSecond, false, true), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.5, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            drive // Reference to this subsystem to set requirements
    );

        NamedCommands.registerCommand("MoveArmScore", autoStructure.moveArmMiddleOuttakeAuto());
        NamedCommands.registerCommand("MoveArmTopIntake", autoStructure.moveArmTopIntakeAuto());
        NamedCommands.registerCommand("MoveArmBottomIntake", autoStructure.moveArmBottomIntakeAuto());
        NamedCommands.registerCommand("Intake", autoStructure.intakeCoralAuto());
        NamedCommands.registerCommand("Outtake", autoStructure.outtakeCoralAuto());
        NamedCommands.registerCommand("OuttakeFast", autoStructure.outtakeFastAuto());
        NamedCommands.registerCommand("StopGripper", autoStructure.stopGripperAuto());

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("Basic Auto", basicAuto());
        // autoChooser.addOption("2 Coral", twoCoralAuto());
        // autoChooser.addOption("3 Coral", threeCoralAuto());
        // autoChooser.addOption("4 Coral", fourCoralAuto());

        
        
    
        SmartDashboard.putData("AutoChooser", autoChooser);
        // autoChooser.addRoutine("aaa", this::followPathAuto);
        // autoChooser.select("aaa");
        // SmartDashboard.putData("Routine" ,autoChooser);

        // autoFactory.bind("MoveArmScore", structure.moveArmMiddleOuttake());
        // autoFactory.bind("Outtake", structure.outtakeCoral());
        // autoFactory.bind("MoveArmIntakeSource", structure.moveArmUpIntake());
        // autoFactory.bind("Intake", structure.IntakeCoral());
        // autoFactory.bind("StopGripper", structure.stopGripper());

    
        
    }
    
    public Command basicAuto(){
        return AutoBuilder.buildAuto("Basic Auto");
    }

    public Command twoCoralAuto(){
        return AutoBuilder.buildAuto("2 Coral Auto");
    }
    
    public Command threeCoralAuto(){
        return AutoBuilder.buildAuto("3 Coral Auto");
    }
    
    public Command fourCoralAuto(){
        return AutoBuilder.buildAuto("4 Coral Auto");
    }

    // public AutoRoutine followPathAuto(){
    //     AutoRoutine routine = autoFactory.newRoutine("followPathAuto");
    //     AutoTrajectory follow = routine.trajectory("New Path");
    //     AutoTrajectory follow1 = routine.trajectory("New Path1");
    //     AutoTrajectory follow2 = routine.trajectory("New Path2");
    //     AutoTrajectory follow3 = routine.trajectory("New Path3");
    //     AutoTrajectory follow4 = routine.trajectory("New Path4");
    //     AutoTrajectory follow5 = routine.trajectory("New Path5");
    //     AutoTrajectory follow6 = routine.trajectory("New Path6");
        

    //     routine.active().onTrue(

            
    //             autoStructure.moveArmMiddleOuttake().andThen(

    //                 Commands.parallel(
    //                 autoStructure.arm.setDesiredAngle(),
    //                 autoStructure.pivot.setDesiredAngle(),
                
    //             Commands.sequence(


                
    //             follow.resetOdometry(),
    //             new InstantCommand(() -> autoStructure.swerve.zeroGyroWithAlliance(), autoStructure.swerve),
                
    //             follow.cmd(),
    //             autoStructure.OuttakeFast().withTimeout(1)
    //                 .andThen(autoStructure.stopGripper())
                
                 
    //             )
    //         )
    //             )
    //     );

    //         follow.done().onTrue(

    //             autoStructure.moveArmUpIntake().andThen(
    //                 Commands.parallel(
    //                     autoStructure.arm.setDesiredAngle(),
    //                     autoStructure.pivot.setDesiredAngle(),

    //                     Commands.sequence(
    //                         follow1.cmd(),
    //                         autoStructure.IntakeCoral().withTimeout(0.7)
    //                         // autoStructure.intakeUntilCoral()
    //                             .andThen(autoStructure.stopGripper())
    //                     )
    //                 )
    //             )
    //             // Commands.sequence(
    //             //     new WaitCommand(1),
    //             //     autoStructure.moveArmUpIntake(),
    //             //     Commands.parallel(
    //             //         autoStructure.arm.setDesiredAngle(),
    //             //         autoStructure.pivot.setDesiredAngle()
    //             //     ).withTimeout(0.2),
    //             //     follow1.cmd(),
    //             //     autoStructure.IntakeCoral().withTimeout(1.5)
    //             //         .andThen(autoStructure.stopGripper())
                    

    //             // )
    //     );

    //     follow1.done().onTrue(
    //         // Commands.sequence(
    //         //     new WaitCommand(1),
    //         //     autoStructure.moveArmMiddleOuttake(),
    //         //     Commands.parallel(
    //         //         autoStructure.arm.setDesiredAngle(),
    //         //         autoStructure.pivot.setDesiredAngle()
    //         //     ).withTimeout(0.1),
    //         //     follow2.cmd(),
    //         //     autoStructure.OuttakeFast().withTimeout(1)
    //         //         .andThen(autoStructure.stopGripper())
    //         // )
    //         new WaitCommand(0.7).andThen(
    //             autoStructure.moveArmMiddleOuttake().andThen(
                
    //         Commands.parallel(
    //             autoStructure.arm.setDesiredAngle(),
    //             autoStructure.pivot.setDesiredAngle(),
            
    //         Commands.sequence(
            
    //         follow2.cmd(),
    //         autoStructure.OuttakeFast().withTimeout(1)
    //             .andThen(autoStructure.stopGripper())
            
             
    //         )
    //     )
    //         )
    //     )
    //     );

    //     follow2.done().onTrue(
            

    //         autoStructure.moveArmUpIntake().andThen(
    //             Commands.parallel(
    //                 autoStructure.arm.setDesiredAngle(),
    //                 autoStructure.pivot.setDesiredAngle(),

    //                 Commands.sequence(
    //                     follow3.cmd(),
    //                     autoStructure.IntakeCoral().withTimeout(0.7)
    //                     // autoStructure.intakeUntilCoral()
    //                         .andThen(autoStructure.stopGripper())
    //                 )
    //             )
    //         )
    //     );
    //     follow3.done().onTrue(
    //         new WaitCommand(0.7).andThen(
    //             autoStructure.moveArmMiddleOuttake().andThen(
                
    //         Commands.parallel(
    //             autoStructure.arm.setDesiredAngle(),
    //             autoStructure.pivot.setDesiredAngle(),
            
    //         Commands.sequence(
            
    //         follow4.cmd(),
    //         autoStructure.OuttakeFast().withTimeout(1)
    //             .andThen(autoStructure.stopGripper())
            
            
             
    //         )
    //     )
    //         )
    //     )
    //     );

    //     follow4.done().onTrue(
    //         autoStructure.moveArmUpIntake().andThen(
    //             Commands.parallel(
    //                 autoStructure.arm.setDesiredAngle(),
    //                 autoStructure.pivot.setDesiredAngle(),

    //                 Commands.sequence(
    //                     follow5.cmd(),
    //                     autoStructure.IntakeCoral().withTimeout(0.7)
    //                     // autoStructure.intakeUntilCoral()
    //                         .andThen(autoStructure.stopGripper())
    //                 )
    //             )
    //         )
    //     );

    //     follow5.done().onTrue(
    //         new WaitCommand(0.7).andThen(
    //             autoStructure.moveArmMiddleOuttake().andThen(
                
    //         Commands.parallel(
    //             autoStructure.arm.setDesiredAngle(),
    //             autoStructure.pivot.setDesiredAngle(),
            
    //         Commands.sequence(
            
    //         follow6.cmd(),
    //         autoStructure.OuttakeFast().withTimeout(1)
    //             .andThen(autoStructure.stopGripper())
            
            
             
    //         )
    //     )
    //         )
    //     )
    //     );
        
    //     return routine;

    // }


    public Command getSelected(){
       return autoChooser.getSelected();
    }


}