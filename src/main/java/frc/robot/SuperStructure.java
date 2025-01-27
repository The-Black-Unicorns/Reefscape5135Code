package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.GripperSubsystem;

public class SuperStructure {
    

    GripperSubsystem gripperSubsystem = new GripperSubsystem();





    public Command ToggleGripper(){

        
        Command selectedMode =
            // gripperSubsystem.isMotorRunning().getAsBoolean() ?
            //     Commands.parallel(gripperSubsystem.stopIntakeCommand(), Commands.print("stopped intake")) :
            //     gripperSubsystem.isCoral()
            // ? 
            //     Commands.parallel(gripperSubsystem.outtakeCommand(), Commands.print("Outtake")) :
            //     Commands.parallel(gripperSubsystem.intakeCommand(), Commands.print("intaked"));
        
                Commands.either(
                    gripperSubsystem.stopGripperCommand(),
                    Commands.either(
                        gripperSubsystem.outtakeCommand(),
                        gripperSubsystem.intakeCommand(),
                        gripperSubsystem::isCoral),
                gripperSubsystem::isMotorRunning);
        System.out.println(selectedMode.getName());
        return selectedMode;
    }

    // public Command ToggleGripper(){
    //     return gripperSubsystem.ToggleGripper();
    // }
 
}
