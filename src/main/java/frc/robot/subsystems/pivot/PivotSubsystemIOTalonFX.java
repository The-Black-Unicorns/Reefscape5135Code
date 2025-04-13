package frc.robot.subsystems.pivot;

import static frc.robot.Constants.PivotConstants.MAX_PIVOT_DEGREES_PER_SECOND;
import static frc.robot.Constants.PivotConstants.MAX_PIVOT_DEGREES_PER_SECOND_SQUARED;
import static frc.robot.Constants.PivotConstants.PIVOT_ENCODER_OFFSET;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KD;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KG;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KI;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KP;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KS;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KV;
import static frc.robot.Constants.PivotConstants.PIVOT_NORMALIZE_OFFSET;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import static frc.robot.Constants.PivotConstants.*;

public class PivotSubsystemIOTalonFX implements PivotSubsystemIO {

    private final TalonFX pivotMotor;

    // private SparkClosedLoopController armController;
    private ProfiledPIDController pivotPIDController;
    private ArmFeedforward pivotFeedforward;
    private TalonFXConfiguration pivotConfig;
    private double currentpivotTargetAngle;
    

    public PivotSubsystemIOTalonFX(int id, int currentLimitAmps, boolean invert, boolean brake){

        pivotConfig = new TalonFXConfiguration();
        
         
        pivotMotor = new TalonFX(id);

        pivotMotor.clearStickyFaults();
         

        pivotConfig.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(currentLimitAmps)
            .withStatorCurrentLimitEnable(true));

        pivotConfig.MotorOutput.Inverted = invert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        pivotConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        pivotConfig.Feedback.FeedbackRotorOffset = PIVOT_ENCODER_OFFSET/360.0;

        pivotMotor.getConfigurator().apply(pivotConfig);
        pivotFeedforward = new ArmFeedforward(PIVOT_MOTOR_KS,PIVOT_MOTOR_KG,PIVOT_MOTOR_KV);
        pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, 
            new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
        pivotPIDController.setTolerance(PIVOT_POSITION_TOLERANCE_DEG);
        currentpivotTargetAngle = pivotMotor.getPosition().getValueAsDouble();

    }

    @Override
    public void setSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        pivotMotor.setVoltage(voltage);
    }

    @Override
    public void setPID(double KP, double KI, double KD) {
        pivotPIDController.setPID(KP, KI, KD);
    }

    @Override
    public void setFF(double KS, double KG, double KV) {
        pivotFeedforward = new ArmFeedforward(KS, KG, KV);
    }

    @Override
    public void setIdleModeBreak() {
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor.getConfigurator().apply(pivotConfig);
    }

    @Override
    public void setIdleModeCoast() {
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(pivotConfig);
    }

    @Override
    public void setTargetAngle(double position) {
        currentpivotTargetAngle = position;
    }

    @Override
    public void moveTargetAngle() {
        double ffVoltage = pivotFeedforward.calculate(pivotMotor.getPosition().getValueAsDouble()*360* Math.PI / 180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
                                                    pivotMotor.getVelocity().getValueAsDouble()*6* Math.PI / 180.0);
        double pidVoltage = pivotPIDController.calculate(pivotMotor.getPosition().getValueAsDouble()*360, currentpivotTargetAngle);
        
        pivotMotor.setVoltage(ffVoltage + pidVoltage);
    }

    @Override
    public void updateInputs(GenericPresicionSystemIOInputs inputs) {
        inputs.Speed = pivotMotor.getVelocity().getValueAsDouble()*6;
        inputs.MotorVoltageLeft = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.MotorTempLeft = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.Angle = pivotMotor.getPosition().getValueAsDouble()*360;
        inputs.TargetAngle = currentpivotTargetAngle;
    }

    @Override
    public void enabledInit(){
        setTargetAngle(pivotMotor.getPosition().getValueAsDouble()/360);
        pivotPIDController.reset(pivotMotor.getPosition().getValueAsDouble()*360);
    }
}
