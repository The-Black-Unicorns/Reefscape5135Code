package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class SuperStructure {

    private final GripperSubsystem gripper;
    final ArmSubsystem arm;
    private final PivotSubsystem pivot;

    public SuperStructure(){

        gripper = new GripperSubsystem();
        arm = new ArmSubsystem();
        pivot = new PivotSubsystem();
        
    }

    public Command ToggleGripper(){

        
        Command selectedMode =        
                Commands.either(
                    gripper.stopGripperCommand(),
                    Commands.either(
                        gripper.outtakeCommand(),
                        gripper.intakeCommand(),
                        gripper::isCoral),
                gripper::isMotorRunning);
        System.out.println(selectedMode.getName());
        return selectedMode;
    }

    public Command moveArmPlewse(DoubleSupplier speed){
        return arm.moveArmManulyCommand(speed);

    }

    public void testPeriodic(){
        gripper.testPeriodic();
        arm.testPeriodic();
        pivot.testPeriodic();
    }
}
