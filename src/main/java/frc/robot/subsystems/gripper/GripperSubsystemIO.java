package frc.robot.subsystems.gripper;

import edu.wpi.first.math.controller.PIDController;

public interface GripperSubsystemIO {

    
    public static class GripperIOInputs {
        public double gripperMotorSpeed = 0; //rots/s
        public double gripperMotorVoltage = 0; //volts
        public double gripperMotorTemp = 0; // celsius
    }

    default public void setGripperMotorSpeed(double speed) {}

    default public void setGripperMotorVoltage(double voltage) {}

    default public void stopGripperMotor() {}

    default public void updateInputs(GripperIOInputs inputs) {}

    default public void setPID(double KP, double KI, double KD) {}
    
    default public void setPID(PIDController pid) {}
    
    default public void gripperAutonInit() {}

} 
    

