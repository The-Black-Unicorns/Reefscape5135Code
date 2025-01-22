package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GripperSubsystem;

public class SuperStructure {
    

    GripperSubsystem gripperSubsystem = new GripperSubsystem();

    public Command ToggleIntake(){

        Command selectedMode = gripperSubsystem.isMotorRunning() ?
        gripperSubsystem.stopIntakeCommand() :
        gripperSubsystem.intakeCommand();
        return selectedMode;
    }
    
}
