package frc.robot.controllers.interfaces;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverInterface {
    
    public DoubleSupplier getLeftX();

    public DoubleSupplier getLeftY();

    public DoubleSupplier getRightX();

    public Trigger isDriving();

    public Trigger getA();


}
