package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

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
        AutoTrajectory follow = routine.trajectory("New Path");
        

        routine.active().onTrue(

            
                autoStructure.moveArmMiddleOuttake().andThen(
                    Commands.parallel(
                    autoStructure.arm.setDesiredAngle(),
                    autoStructure.pivot.setDesiredAngle(),
                
                Commands.sequence(
                
                follow.resetOdometry(),
                follow.cmd(),
                autoStructure.OuttakeFast().withTimeout(5)
                 
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