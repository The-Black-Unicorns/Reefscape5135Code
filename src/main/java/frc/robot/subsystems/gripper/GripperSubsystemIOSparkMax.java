package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;

public class GripperSubsystemIOSparkMax implements GripperSubsystemIO {
    SparkMax gripperMotor;
    SparkMaxConfig configs;
    VelocityVoltage velocityVoltage;

    
    public GripperSubsystemIOSparkMax(int deviceID, int currentLimitAmps, boolean invert, boolean brake) {
        gripperMotor = new SparkMax(deviceID, MotorType.kBrushless);
        gripperMotor.clearFaults();
        configs = new SparkMaxConfig();
        // configs.smartCurrentLimit(currentLimitAmps);
        configs.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);
        configs.inverted(invert);

        gripperMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        velocityVoltage = new VelocityVoltage(0);

    }

    @Override
    public void setGripperMotorSpeed(double speed) {
        gripperMotor.set(speed);;
    }

    @Override
    public void setGripperMotorVoltage(double voltage) {
        gripperMotor.setVoltage(voltage);
    }

    @Override
    public void stopGripperMotor() {
        gripperMotor.set(0);
    }

    @Override
    public void updateInputs(GripperIOInputs inputs) {
        inputs.gripperMotorSpeed = gripperMotor.getEncoder().getVelocity();
        inputs.gripperMotorVoltage = gripperMotor.getBusVoltage();
        inputs.gripperMotorTemp = gripperMotor.getMotorTemperature();
    }

    @Override
    public void setPID(double KP, double KI, double KD) {
        configs.closedLoop.p(KP);
        configs.closedLoop.i(KI);
        configs.closedLoop.d(KD);
        gripperMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    @Override
    public void setPID(PIDController pid) {
        configs.closedLoop.p(pid.getP());
        configs.closedLoop.i(pid.getI());
        configs.closedLoop.d(pid.getD());
        gripperMotor.configure(configs, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
