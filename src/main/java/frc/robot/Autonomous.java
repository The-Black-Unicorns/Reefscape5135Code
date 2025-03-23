package frc.robot;

import com.fasterxml.jackson.databind.introspect.AnnotatedMethod;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveSubsystem;

import static frc.robot.Constants.AutoConstants;
public class Autonomous {
    private final SwerveSubsystem drive;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;
    private final SuperStructure autoStructure;
    

    public Autonomous(SuperStructure structure){
        drive = structure.swerve;
        autoChooser = new AutoChooser();
        autoStructure = structure;
        

        autoFactory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            false,
            drive
        );
        
        autoChooser.addRoutine("aaa", this::followPathAuto);
        autoChooser.select("aaa");
        SmartDashboard.putData("Routine" ,autoChooser);

        autoFactory.bind("MoveArmScore", structure.moveArmMiddleOuttake());
        autoFactory.bind("Outtake", structure.outtakeCoral());
        autoFactory.bind("MoveArmIntakeSource", structure.moveArmUpIntake());
        autoFactory.bind("Intake", structure.IntakeCoral());
        autoFactory.bind("StopGripper", structure.stopGripper());
    }
    
    
    

    

    public AutoRoutine followPathAuto(){
        AutoRoutine routine = autoFactory.newRoutine("followPathAuto");
        AutoTrajectory follow0 = routine.trajectory("ml to 2b");
        AutoTrajectory follow1 = routine.trajectory("2b to source");
        AutoTrajectory follow2 = routine.trajectory("source to 1b");
        AutoTrajectory follow3 = routine.trajectory("1b to source");
        AutoTrajectory follow4 = routine.trajectory("source to 1a");
        AutoTrajectory follow5 = routine.trajectory("1a to source");
        AutoTrajectory follow6 = routine.trajectory("source to 1a");
        

        routine.active().onTrue(

            
                autoStructure.moveArmMiddleOuttake().andThen(

                    Commands.parallel(
                    autoStructure.arm.setDesiredAngle(),
                    autoStructure.pivot.setDesiredAngle(),
                
                Commands.sequence(


                
                follow0.resetOdometry(),
                new InstantCommand(() -> autoStructure.swerve.zeroGyroWithAlliance(), autoStructure.swerve),
                
                follow0.cmd(),
                autoStructure.outtakeCoral().withTimeout(AutoConstants.AUTO_OUTTAKE_TIME)
                    .andThen(autoStructure.stopGripper())
                
                 
                )
            )
                )
        );

            follow0.done().onTrue(

                autoStructure.moveArmUpIntake().andThen(
                    Commands.parallel(
                        autoStructure.arm.setDesiredAngle(),
                        autoStructure.pivot.setDesiredAngle(),

                        Commands.sequence(
                            follow1.cmd(),
                            autoStructure.IntakeCoral().withTimeout(AutoConstants.AUTO_INTAKE_TIME)
                            // autoStructure.intakeUntilCoral()
                                .andThen(autoStructure.stopGripper())
                        )
                    )
                )
                // Commands.sequence(
                //     new WaitCommand(1),
                //     autoStructure.moveArmUpIntake(),
                //     Commands.parallel(
                //         autoStructure.arm.setDesiredAngle(),
                //         autoStructure.pivot.setDesiredAngle()
                //     ).withTimeout(0.2),
                //     follow1.cmd(),
                //     autoStructure.IntakeCoral().withTimeout(1.5)
                //         .andThen(autoStructure.stopGripper())
                    

                // )
        );

        follow1.done().onTrue(
            // Commands.sequence(
            //     new WaitCommand(1),
            //     autoStructure.moveArmMiddleOuttake(),
            //     Commands.parallel(
            //         autoStructure.arm.setDesiredAngle(),
            //         autoStructure.pivot.setDesiredAngle()
            //     ).withTimeout(0.1),
            //     follow2.cmd(),
            //     autoStructure.OuttakeFast().withTimeout(1)
            //         .andThen(autoStructure.stopGripper())
            // )
            new WaitCommand(AutoConstants.AUTO_TIME_BETWEEN_AUTOS).andThen(
                autoStructure.moveArmMiddleOuttake().andThen(
                
            Commands.parallel(
                autoStructure.arm.setDesiredAngle(),
                autoStructure.pivot.setDesiredAngle(),
            
            Commands.sequence(
            
            follow2.cmd(),
            autoStructure.outtakeCoral().withTimeout(AutoConstants.AUTO_OUTTAKE_TIME)
                .andThen(autoStructure.stopGripper())
            
             
            )
        )
            )
        )
        );

        follow2.done().onTrue(
            

            autoStructure.moveArmUpIntake().andThen(
                Commands.parallel(
                    autoStructure.arm.setDesiredAngle(),
                    autoStructure.pivot.setDesiredAngle(),

                    Commands.sequence(
                        follow3.cmd(),
                        autoStructure.IntakeCoral().withTimeout(AutoConstants.AUTO_INTAKE_TIME)
                        // autoStructure.intakeUntilCoral()
                            .andThen(autoStructure.stopGripper())
                    )
                )
            )
        );
        follow3.done().onTrue(
            new WaitCommand(AutoConstants.AUTO_TIME_BETWEEN_AUTOS).andThen(
                autoStructure.moveArmMiddleOuttake().andThen(
                
            Commands.parallel(
                autoStructure.arm.setDesiredAngle(),
                autoStructure.pivot.setDesiredAngle(),
            
            Commands.sequence(
            
            follow4.cmd(),
            autoStructure.outtakeCoral().withTimeout(AutoConstants.AUTO_OUTTAKE_TIME)
                .andThen(autoStructure.stopGripper())
            
            
             
            )
        )
            )
        )
        );

        follow4.done().onTrue(
            autoStructure.moveArmUpIntake().andThen(
                Commands.parallel(
                    autoStructure.arm.setDesiredAngle(),
                    autoStructure.pivot.setDesiredAngle(),

                    Commands.sequence(
                        follow5.cmd(),
                        autoStructure.IntakeCoral().withTimeout(AutoConstants.AUTO_INTAKE_TIME)
                        // autoStructure.intakeUntilCoral()
                            .andThen(autoStructure.stopGripper())
                    )
                )
            )
        );

        follow5.done().onTrue(
            new WaitCommand(AutoConstants.AUTO_TIME_BETWEEN_AUTOS).andThen(
                autoStructure.moveArmMiddleOuttake().andThen(
                
            Commands.parallel(
                autoStructure.arm.setDesiredAngle(),
                autoStructure.pivot.setDesiredAngle(),
            
            Commands.sequence(
            
            follow6.cmd(),
            autoStructure.outtakeCoral().withTimeout(AutoConstants.AUTO_OUTTAKE_TIME)
                .andThen(autoStructure.stopGripper())
            
            
             
            )
        )
            )
        )
        );
        

        
            
        
            
            
        
            
        

        return routine;

    }


    
    public Command getSelected(){
       return autoChooser.selectedCommandScheduler();
    }


}