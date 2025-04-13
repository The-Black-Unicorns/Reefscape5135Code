package frc.robot.subsystems.arm;

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
import static frc.robot.Constants.Arm.*;


public class ArmSubsystemIOSparkMax implements ArmSubsystemIO {
    

    private final SparkMax armMotorR,armMotorL;
    private AbsoluteEncoder armEncoder; 
    // private SparkClosedLoopController armController;
    private ProfiledPIDController armPIDController;
    private ArmFeedforward armFeedforward;
    private SparkMaxConfig armConfigL,armConfigR;
    private double currentArmTargetAngle;
    

    public ArmSubsystemIOSparkMax(int armL_ID, int armR_ID, int currentLimitAmps, boolean invert, boolean brake){
        armConfigL = new SparkMaxConfig();
        armConfigR = new SparkMaxConfig();
         
        armMotorR = new SparkMax(armR_ID, MotorType.kBrushless);
        armMotorL = new SparkMax(armL_ID, MotorType.kBrushless);
        armMotorR.clearFaults();
        armMotorL.clearFaults();
         
        armConfigR.absoluteEncoder.positionConversionFactor(360);
        armConfigR.absoluteEncoder.velocityConversionFactor(6);

        armConfigR.smartCurrentLimit(currentLimitAmps);

        armConfigR.inverted(invert);
        armConfigR.idleMode(brake ? SparkMaxConfig.IdleMode.kBrake : SparkMaxConfig.IdleMode.kCoast);

        armConfigL.apply(armConfigR);
        armConfigR.follow(armMotorL, true);
        armConfigL.absoluteEncoder.zeroOffset(ARM_ENCODER_OFFSET/360.0);
        armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armFeedforward = new ArmFeedforward(ARM_KS,ARM_KG,ARM_KV);
        armEncoder = armMotorL.getAbsoluteEncoder();
        armPIDController = new ProfiledPIDController(ARM_KP, ARM_KI, ARM_KD, 
            new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELARATION));
        armPIDController.setTolerance(ARM_POSITION_TOLERANCE_DEG);
        currentArmTargetAngle = armEncoder.getPosition();
        armPIDController.enableContinuousInput(0, 360);

    }

    @Override
    public void setSpeed(double speed) {
        armMotorL.set(speed);
    }

    @Override
    public void setVoltage(double voltage) {
        armMotorL.setVoltage(voltage);
    }

    @Override
    public void setPID(double KP, double KI, double KD) {
        armPIDController.setPID(KP, KI, KD);
    }

    @Override
    public void setFF(double KS, double KG, double KV) {
        armFeedforward = new ArmFeedforward(KS, KG, KV);
    }

    @Override
    public void setIdleModeBreak() {
        armConfigR.idleMode(IdleMode.kBrake);
        armConfigL.idleMode(IdleMode.kBrake);
        armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIdleModeCoast() {
        armConfigR.idleMode(IdleMode.kCoast);
        armConfigL.idleMode(IdleMode.kCoast);
        armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTargetAngle(double position) {
        System.out.println("oihtethhtjrr");
        currentArmTargetAngle = position;
    }

    @Override
    public void moveTargetAngle() {
        double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0 - ARM_NORMALIZE_OFFSET* Math.PI / 180.0,
                                                    armEncoder.getVelocity()* Math.PI / 180.0);
        double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), currentArmTargetAngle);
        
        armMotorL.setVoltage(ffVoltage + pidVoltage);
    }

    @Override
    public void updateInputs(GenericPresicionSystemIOInputs inputs) {
        inputs.Speed = armEncoder.getVelocity();
        inputs.MotorVoltageLeft = armMotorL.getBusVoltage();
        inputs.MotorVoltageRight = armMotorR.getBusVoltage();
        inputs.MotorTempLeft = armMotorL.getMotorTemperature();
        inputs.MotorTempRight = armMotorR.getMotorTemperature();
        inputs.Angle = armEncoder.getPosition();
        inputs.TargetAngle = currentArmTargetAngle;
    }

    @Override
    public void enabledInit(){
        setTargetAngle(armEncoder.getPosition());
        armPIDController.reset(armEncoder.getPosition());
    }

}
