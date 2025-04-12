package frc.robot.subsystems.gripper;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public class GripperSubsystemIOTalonFX implements GripperSubsystemIO {
    TalonFX gripperMotor;
    TalonFXConfiguration configs;
    VelocityVoltage velocityVoltage;

    private final StatusSignal<AngularVelocity> velocityRps;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Temperature> tempCelsius;
    
    
    public GripperSubsystemIOTalonFX(int deviceID, int currentLimitAmps, boolean invert, boolean brake) {
        gripperMotor = new TalonFX(0);
        gripperMotor.clearStickyFaults();
        configs = new TalonFXConfiguration();
        configs.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimitAmps)
            .withStatorCurrentLimitEnable(true));
        configs.MotorOutput.Inverted = invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        
        gripperMotor.getConfigurator().apply(configs);

        velocityVoltage = new VelocityVoltage(0);

        velocityRps = gripperMotor.getVelocity();
        appliedVoltage = gripperMotor.getMotorVoltage();
        tempCelsius = gripperMotor.getDeviceTemp();

        
    }

    @Override
    public void setGripperMotorSpeed(double speed) {
        gripperMotor.set(speed);
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
        inputs.gripperMotorSpeed = velocityRps.getValueAsDouble();
        inputs.gripperMotorVoltage = appliedVoltage.getValueAsDouble();
        inputs.gripperMotorTemp = tempCelsius.getValueAsDouble();
    }

    @Override
    public void setPID(double KP, double KI, double KD) {
        configs.Slot0.kP = KP;
        configs.Slot0.kI = KI;
        configs.Slot0.kD = KD;
        gripperMotor.getConfigurator().apply(configs);
    }

    @Override
    public void setPID(PIDController pid) {
        configs.Slot0.kP = pid.getP();
        configs.Slot0.kI = pid.getI();
        configs.Slot0.kD = pid.getD();
        gripperMotor.getConfigurator().apply(configs);
    }




}
