package frc.robot.controllers.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.interfaces.DriverInterface;

import static frc.robot.Constants.ControllerConstants.*;

public class DriveController implements DriverInterface {
    private GenericHID controller;
    
    public DriveController(int id) {
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
        return () -> controller.getRawAxis(0);
    }

    @Override
    public DoubleSupplier getXSpeed(){
        return () -> controller.getRawAxis(4);
    }

    @Override
    public DoubleSupplier getYSpeed(){
        return () -> controller.getRawAxis(5);
    }

    @Override
    public Trigger resetGyroButton(){
        return new Trigger(()->controller.getRawButton(3));
    }

    @Override
    public Trigger raiseArm(){
        return new Trigger(() -> controller.getRawButton(5));
    }

    @Override
    public Trigger lowerArm(){
        return new Trigger(() -> controller.getRawAxis(2) > BACK_BUTTONS_DEADBAND);
    }

    public Trigger outtakeCoral(){
        return new Trigger(()->controller.getRawButton(6));
    }

    public Trigger intakeCoral(){
        return new Trigger(() -> controller.getRawAxis(3) > BACK_BUTTONS_DEADBAND);
    }

    public Trigger climbMode(){
        return new Trigger(() -> (controller.getRawButton(8) && controller.getRawButton(7)));
    }

    public Trigger shouldArmMoveTrigger(){
        return new Trigger(() -> controller.getRawAxis(1) > 0.1 || controller.getRawAxis(1) < -0.1);
    }
    public DoubleSupplier getArmSpeed(){
        return () -> controller.getRawAxis(1);
    }
}
