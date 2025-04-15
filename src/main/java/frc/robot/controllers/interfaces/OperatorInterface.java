package frc.robot.controllers.interfaces;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorInterface {


    public Trigger setArmLowAngleButton();

    public Trigger setArmMidAngleButton();

    public Trigger setArmTopAngleButton();

    public Trigger setArmClimbingAngleButton();

    // public Trigger setArmTopAngleOuttake();

    public Trigger outtakeCoralButton();

    public Trigger intakeCoralButton();

    public Trigger isGripperActiveButton();

    public Trigger outtakeFastCoralButton();

    public Trigger setArmAlgaeAngleButton();
}

