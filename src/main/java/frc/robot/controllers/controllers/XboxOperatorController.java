package frc.robot.controllers.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.interfaces.DriverInterface;
import frc.robot.controllers.interfaces.OperatorInterface;

import static frc.robot.Constants.ControllerConstants.*;

public class XboxOperatorController implements OperatorInterface {
    private GenericHID controller;
    
    public XboxOperatorController(int id) {
        controller = new GenericHID(id);
    } //add stick deadband

    @Override
    public Trigger isGripperActiveButton() {
        return new Trigger(() -> (intakeCoralButton().getAsBoolean() || outtakeCoralButton().getAsBoolean()));
    }

    @Override
    public Trigger outtakeCoralButton(){
        return new Trigger(()->controller.getRawButton(6));
    }

    @Override
    public Trigger intakeCoralButton(){
        return new Trigger(() -> controller.getRawAxis(3) > BACK_BUTTONS_DEADBAND);
    }

    @Override
    public Trigger setArmLowAngleButton() {
        
        return new Trigger(() -> controller.getRawButton(1));
    }

    @Override
    public Trigger setArmMidAngleButton() {
        return new Trigger(() -> controller.getRawButton(3));
    }

    @Override
    public Trigger setArmTopAngleButton() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    @Override
    public Trigger outtakeFastCoralButton() {
        
        return new Trigger(() -> controller.getRawButton(5));
    }

    @Override
    public Trigger setArmClimbingAngleButton() {
        return new Trigger(() -> controller.getRawButton(7) && controller.getRawButton(8));
    }

//     @Override
//     public Trigger setArmTopAngleOuttake() {
//         return new Trigger(() -> controller.getRawButton(1));
//         }
}

