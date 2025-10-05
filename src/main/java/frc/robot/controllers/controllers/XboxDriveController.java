package frc.robot.controllers.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.interfaces.DriverInterface;

public class XboxDriveController implements DriverInterface {

    private XboxController controller;
    //this is qx controller!!!

    public XboxDriveController(int id) {
        controller = new XboxController(id);
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
        return (() -> -controller.getRightX()*Math.abs(controller.getRightX()) / 1.3);
    }

    @Override
    public DoubleSupplier getXSpeed(){
        return (() -> controller.getLeftX() * Math.abs(controller.getLeftX()));
    }

    @Override
    public DoubleSupplier getYSpeed(){
        return () -> -controller.getLeftY()*Math.abs(controller.getLeftY());
    }

    @Override
    public Trigger resetGyroButton(){
        return new Trigger(()->(controller.getRawButton(7) && controller.getRawButton(8)));
    }


    @Override
    public DoubleSupplier getSpeedPotentiometer() {
        return () -> 1;
    }

    public Trigger isGripperActive() {
        return new Trigger(() -> controller.getLeftBumperButton());
    }

    public Trigger getIntakeMode() {
        return new Trigger(() -> controller.getRightBumperButton());
    }
}
