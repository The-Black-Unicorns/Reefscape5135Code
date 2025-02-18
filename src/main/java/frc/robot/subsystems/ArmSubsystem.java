package  frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Arm;

import static frc.robot.Constants.Arm.*;

import java.util.function.DoubleSupplier;

public class ArmSubsystem extends SubsystemBase  {

    
    private final SparkMax armMotorR,armMotorL;
    private AbsoluteEncoder armEncoder; 
    // private SparkClosedLoopController armController;
    private ProfiledPIDController armPIDController;
    private ArmFeedforward armFeedforward;
    private SparkMaxConfig armConfigL,armConfigR;
    private double currentArmTargetAngle;
    
    private double KP, KI, KD;
    private double KS, KG, KV;

    public ArmSubsystem(){
         armConfigL = new SparkMaxConfig();
         armConfigR = new SparkMaxConfig();
         
         armMotorR = new SparkMax(RIGHT_ARM_MOTOR, MotorType.kBrushless);
         armMotorL = new SparkMax(LEFT_ARM_MOTOR, MotorType.kBrushless);
         
         armConfigR.absoluteEncoder.positionConversionFactor(360);
         armConfigR.absoluteEncoder.velocityConversionFactor(6);

         armConfigR.inverted(false);
         armConfigR.idleMode(IdleMode.kBrake);
        //  armConfigR.closedLoop.
        //  feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        //  .pid(Arm.ARM_KP, Arm.ARM_KI, Arm.ARM_KD)
        //  .maxMotion.maxVelocity(Arm.ARM_MAX_VELOCITY);
        //  armConfigR.closedLoop.maxMotion.maxAcceleration(Arm.ARM_MAX_ACCELARATION);
         armConfigL.apply(armConfigR);
         armConfigR.follow(armMotorL, true);
         armConfigL.absoluteEncoder.zeroOffset(Arm.ARM_ENCODER_OFFSET/360.0);
         armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //  armController = armMotorR.getClosedLoopController();
         armFeedforward = new ArmFeedforward(Arm.ARM_KS,Arm.ARM_KG,Arm.ARM_KV);
         armEncoder = armMotorL.getAbsoluteEncoder();
        armPIDController = new ProfiledPIDController(ARM_KP, ARM_KI, ARM_KD, 
            new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELARATION));
        armPIDController.setTolerance(Arm.ARM_POSITION_TOLERANCE_DEG);
        currentArmTargetAngle = armEncoder.getPosition();
        armPIDController.enableContinuousInput(0, 360);

        KP = Arm.ARM_KP;
        KI = Arm.ARM_KI;
        KD = Arm.ARM_KD;

        KS = Arm.ARM_KS;
        KG = Arm.ARM_KG;
        KV = Arm.ARM_KV;

    }

    public TrapezoidProfile.State getCurrentState(){
        return new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
    }

    public void setIdleModeBreak(){
        armConfigR.idleMode(IdleMode.kBrake);
        armConfigL.idleMode(IdleMode.kBrake);
        armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public void setIdleModeCoast(){
        armConfigR.idleMode(IdleMode.kCoast);
        armConfigL.idleMode(IdleMode.kCoast);
        armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        System.out.println("set coast");
    }
    public double getArmAngle(){
        return armEncoder.getPosition();
    }

    public double getArmVelocity(){
        return armEncoder.getVelocity();
    }

    // public ProxyCommand controlArmMotorProxy(double angle){
    //     return new ProxyCommand(new RunCommand(() -> setArmAngle(angle),this));
    // }


    public Command setDesiredAngle() {
        return new RunCommand(() -> setArmAngle(this::getDesiredArmAngle), this);
    }


    public void setArmAngle(DoubleSupplier targetAngleDegrees){


        double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0 - ARM_NORMALIZE_OFFSET* Math.PI / 180.0,
                                                    armEncoder.getVelocity()* Math.PI / 180.0);
        double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), targetAngleDegrees.getAsDouble()
        );
        
        armMotorL.setVoltage(ffVoltage + pidVoltage);

    }

    public void setArmManualSpeed(double speed){

        // armMotorR.set(speed);
        double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0,
        armEncoder.getVelocity()* Math.PI / 180.0);

        armMotorL.setVoltage(speed+ffVoltage);

    }

    public Command moveArmManulyCommand(DoubleSupplier speed){
        return new RunCommand(() -> setArmAngle(speed), this);
    }

    public void setArmTargetAngle(double angleDeg){
        currentArmTargetAngle = angleDeg;
    }

    public Command setDesiredAngleDeg(double desiredAngleDeg){
        return runOnce(()->setArmTargetAngle(desiredAngleDeg));
    }

    public void armEnabledInit(){
        setArmTargetAngle(armEncoder.getPosition());
        armPIDController.reset(armEncoder.getPosition());
    }

    public Command armEnabledInitCommand(){
        return runOnce(()->armEnabledInit());
    }

    private double getDesiredArmAngle(){
        return currentArmTargetAngle;
    }
    
    
    

    public void periodic(){
        double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0,
        armEncoder.getVelocity()* Math.PI / 180.0);
        // double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), 20.6);
        // SmartDashboard.putNumber("Arm/pidVoltage", armPIDController.calculate(armEncoder.getPosition(), 60));
        SmartDashboard.putNumber("Arm/ArmAngle", getArmAngle());
        SmartDashboard.putNumber("Arm/armSpeed", getArmVelocity());
        SmartDashboard.putNumber("Arm/armVoltageR", armMotorR.getBusVoltage());
        SmartDashboard.putNumber("Arm/armVoltageL", armMotorL.getBusVoltage());
        SmartDashboard.putNumber("Arm/desiredAngle", currentArmTargetAngle);

        SmartDashboard.putNumber("Arm/ffVoltage", ffVoltage);
        // SmartDashboard.putNumber("Arm/pidVoltage", pidVoltage);

    }
    public void testPeriodic(){
        double newKP = SmartDashboard.getNumber("Arm/armKp", KP);
        double newKI = SmartDashboard.getNumber("Arm/armKi", KI);
        double newKD = SmartDashboard.getNumber("Arm/armKd", KD);

        if(newKP != KP || newKI != KI || newKD != KD){
            KP = newKP;
            KI = newKI;
            KD = newKD;

            armConfigR.closedLoop.
            feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(newKP, newKI, newKD);

            armConfigL.closedLoop.
            feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(newKP, newKI, newKD);

            armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        }
        double newKS = SmartDashboard.getNumber("Arm/armKs", KS);
        double newKG = SmartDashboard.getNumber("Arm/armKg", KG);
        double newKV = SmartDashboard.getNumber("Arm/armKv", KV);

        if(newKS != KS || newKG != KG || newKV != KV){
            KS = newKS;
            KG = newKG;
            KV = newKV;

            armFeedforward = new ArmFeedforward(KS, KG, KV);
        }
    }

    
}
