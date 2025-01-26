package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;

public class SuperStructure {
    

    GripperSubsystem gripperSubsystem = new GripperSubsystem();

    public Command ToggleGripper(){

        Command selectedMode = gripperSubsystem.isMotorRunning() ?
            gripperSubsystem.stopIntakeCommand() :
            gripperSubsystem.isCoral() ? 
                gripperSubsystem.outtakeCommand() :
                gripperSubsystem.intakeCommand();
        
        return selectedMode;
    }
    
}
