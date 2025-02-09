package frc.robot;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public class Autonomous {
    private final SwerveSubsystem drive;
    private final AutoFactory autoFactory;
    private final AutoChooser autoChooser;
    // private TrajectoryLogger<SwerveSample> logger;

    public Autonomous(){
        drive = new SwerveSubsystem();
        autoChooser = new AutoChooser();
        

        autoFactory = new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            false,
            drive
            // logger
        );
        
        autoChooser.addRoutine("aaa", this::followPathAuto);
        autoChooser.select("aaa");
        SmartDashboard.putData("Routine" ,autoChooser);
    }
    
    
    

    

    public AutoRoutine followPathAuto(){
        AutoRoutine routine = autoFactory.newRoutine("followPathAuto");
        AutoTrajectory follow = routine.trajectory("New Path");

        routine.active().onTrue(
            Commands.sequence(
                follow.resetOdometry(),
                follow.cmd()
                
            )
        );
        return routine;
        // );
        // return Commands.sequence(
        //     autoFactory.resetOdometry("New Path"),
        //     follow
        // );
    }


    
    public Command getSelected(){
       return autoChooser.selectedCommandScheduler();
    }


}
