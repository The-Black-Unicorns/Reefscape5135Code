// package frc.robot;

// import choreo.auto.AutoChooser;
// import choreo.auto.AutoFactory;
// import choreo.auto.AutoRoutine;
// import choreo.auto.AutoTrajectory;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.subsystems.swerveSubsystem.SwerveSubsystem;

// public class Autonomous {
//     private final SwerveSubsystem drive;
//     private final AutoFactory autoFactory;
//     private final AutoChooser autoChooser;
    

//     public Autonomous(){
//         drive = new SwerveSubsystem();
//         autoChooser = new AutoChooser();
        

//         autoFactory = new AutoFactory(
//             drive::getPose,
//             drive::setPose,
//             drive::followTrajectory,
//             false,
//             drive
//         );
        
//         autoChooser.addRoutine("aaa", this::followPathAuto);
//         autoChooser.select("aaa");
//         SmartDashboard.putData("Routine" ,autoChooser);
//     }
    
    
    

    

//     public AutoRoutine followPathAuto(){
//         AutoRoutine routine = autoFactory.newRoutine("followPathAuto");
//         AutoTrajectory follow = routine.trajectory("New Path");

//         routine.active().onTrue(
//             Commands.sequence(
//                 follow.resetOdometry(),
//                 follow.cmd()
                
//             )
//         );
//         return routine;

//     }


    
//     public Command getSelected(){
//        return autoChooser.selectedCommandScheduler();
//     }


// }