package frc.robot.controllers.controllers;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.interfaces.DriverInterface;
import frc.robot.controllers.interfaces.OperatorInterface;

import static frc.robot.Constants.ControllerConstants.*;

public class QxDriveController implements DriverInterface{
    private GenericHID controller;
    //this is qx controller!!!

    public QxDriveController(int id) {
        controller = new GenericHID(id);
    } //add stick deadband

    @Override
    public Trigger isDriving() {
        if (getXSpeed().getAsDouble() != 0 ||
        getYSpeed().getAsDouble() != 0 ||   
        getRotationSpeed().getAsDouble() != 0){
            return new Trigger(() -> true); 
        }
        else {return new Trigger(() -> false);}
    }

    @Override
    public DoubleSupplier getRotationSpeed(){
        return (() -> -controller.getRawAxis(0)*Math.abs(controller.getRawAxis(0)) * (controller.getRawAxis(5) + 1) / 2.0);
    }

    @Override
    public DoubleSupplier getXSpeed(){
        return (() -> controller.getRawAxis(3) * Math.abs(controller.getRawAxis(3)));
    }

    @Override
    public DoubleSupplier getYSpeed(){
        return () -> -controller.getRawAxis(2)*Math.abs(controller.getRawAxis(2));
    }


    @Override
    public Trigger resetGyroButton(){
        // return new Trigger(()->controller.getRawButton(2));
        return new Trigger(() -> controller.getRawAxis(6) > 0);
    }


    @Override
    public DoubleSupplier getSpeedPotentiometer() {
        return () -> controller.getRawAxis(4) * controller.getRawAxis(4);
    }

    public Trigger isGripperActive() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    public Trigger getIntakeMode() {
        return new Trigger(() -> controller.getRawButton(6));
    }

}
