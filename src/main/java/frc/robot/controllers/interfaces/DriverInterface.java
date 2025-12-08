package frc.robot.controllers.interfaces;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public interface DriverInterface {

  public Trigger isDriving();

  public DoubleSupplier getRotationSpeed();

  public DoubleSupplier getXSpeed();

  public DoubleSupplier getYSpeed();

  public Trigger resetGyroButton();

  // public Trigger raiseArmOne();

  // public Trigger lowerArmOne();

  // public Trigger outtakeCoral();

  // public Trigger intakeCoral();

  // public Trigger isGripperActive();

  // public Trigger climbMode();

  // public Trigger shouldArmMoveTrigger();

  // public DoubleSupplier getArmSpeed();

  // public Trigger getIntakeMode();

  // public Trigger setIntakeDown();

  // public Trigger setIntakeUp();

  public DoubleSupplier getSpeedPotentiometer();
}
