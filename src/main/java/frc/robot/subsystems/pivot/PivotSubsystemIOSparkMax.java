package frc.robot.subsystems.pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


import static frc.robot.Constants.PivotConstants.*; // Ensure this import matches the actual location of PivotSubsystemIO

public class PivotSubsystemIOSparkMax implements PivotSubsystemIO{
    
    private final SparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder; 
    // private SparkClosedLoopController armController;
    private ProfiledPIDController pivotPIDController;
    private ArmFeedforward pivotFeedforward;
    private SparkMaxConfig pivotConfig;
    private double currentpivotTargetAngle;
    

    public PivotSubsystemIOSparkMax(int id, int currentLimitAmps, boolean invert, boolean brake){
        pivotConfig = new SparkMaxConfig();
         
        pivotMotor = new SparkMax(id, MotorType.kBrushless);

        pivotMotor.clearFaults();
         
        pivotConfig.absoluteEncoder.positionConversionFactor(360);
        pivotConfig.absoluteEncoder.velocityConversionFactor(6);

        pivotConfig.smartCurrentLimit(currentLimitAmps);

        pivotConfig.inverted(invert);
        pivotConfig.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

        pivotConfig.absoluteEncoder.zeroOffset(PIVOT_ENCODER_OFFSET/360.0);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotFeedforward = new ArmFeedforward(PIVOT_MOTOR_KS,PIVOT_MOTOR_KG,PIVOT_MOTOR_KV);
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, 
            new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
        pivotPIDController.setTolerance(PIVOT_POSITION_TOLERANCE_DEG);
        currentpivotTargetAngle = pivotEncoder.getPosition();

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
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIdleModeCoast() {
        pivotConfig.idleMode(IdleMode.kCoast);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTargetAngle(double position) {
        currentpivotTargetAngle = position;
    }

    @Override
    public void moveTargetAngle() {
        double ffVoltage = pivotFeedforward.calculate(pivotEncoder.getPosition()* Math.PI / 180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
                                                    pivotEncoder.getVelocity()* Math.PI / 180.0);
        double pidVoltage = pivotPIDController.calculate(pivotEncoder.getPosition(), currentpivotTargetAngle);
        
        pivotMotor.setVoltage(ffVoltage + pidVoltage);
    }

    @Override
    public void updateInputs(GenericPresicionSystemIOInputs inputs) {
        inputs.Speed = pivotEncoder.getVelocity();
        inputs.MotorVoltageLeft = pivotMotor.getBusVoltage();
        inputs.MotorTempLeft = pivotMotor.getMotorTemperature();
        inputs.Angle = pivotEncoder.getPosition();
        inputs.TargetAngle = currentpivotTargetAngle;
    }

    @Override
    public void enabledInit(){
        setTargetAngle(pivotEncoder.getPosition());
        pivotPIDController.reset(pivotEncoder.getPosition());
    }
}
