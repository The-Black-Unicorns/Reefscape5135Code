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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Arm;

import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase  {

    
    private SparkMax armMotorR,armMotorL;
    private AbsoluteEncoder armEncoder; 
    private SparkClosedLoopController armController;
    private ArmFeedforward armFeedforward;
    private SparkMaxConfig armConfigL,armConfigR;
    
    private double KP, KI, KD;
    private double KS, KG, KV;

    public ArmSubsystem(){
         armConfigL = new SparkMaxConfig();
         armConfigR = new SparkMaxConfig();
         
         armMotorR = new SparkMax(RIGHT_ARM_MOTOR, MotorType.kBrushless);
         armMotorR = new SparkMax(LEFT_ARM_MOTOR, MotorType.kBrushless);
         
         armConfigR.absoluteEncoder.positionConversionFactor(360);
         armConfigR.absoluteEncoder.velocityConversionFactor(360/60);
         armConfigR.absoluteEncoder.zeroOffset(Arm.ARM_ENCODER_OFFSET);
         armConfigR.inverted(false);
         armConfigR.idleMode(IdleMode.kBrake);
         armConfigR.closedLoop.
         feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
         .pid(Arm.ARM_KP, Arm.ARM_KI, Arm.ARM_KD)
         .maxMotion.maxVelocity(Arm.ARM_MAX_VELOCITY);
         armConfigR.closedLoop.maxMotion.maxAcceleration(Arm.ARM_MAX_ACCELARATION);
         armConfigL.apply(armConfigR);
         armConfigL.follow(armMotorR, true);
         armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
         armController = armMotorR.getClosedLoopController();
         armFeedforward = new ArmFeedforward(Arm.ARM_KS,Arm.ARM_KG,Arm.ARM_KV);
         armEncoder=armMotorR.getAbsoluteEncoder();


        KP = Arm.ARM_KP;
        KI = Arm.ARM_KI;
        KD = Arm.ARM_KD;

        KS = Arm.ARM_KS;
        KG = Arm.ARM_KG;
        KV = Arm.ARM_KV;

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
        armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public double getArmAngle(){
        return armEncoder.getPosition();
    }

    public double getArmVelocity(){
        return armEncoder.getVelocity();
    }

    public ProxyCommand controlArmMotorProxy(double angle){
        return new ProxyCommand(new RunCommand(() -> setArmAngle(angle),this));
    }

    public Command controlArmMotor(double angle) {
        return new RunCommand(() -> setArmAngle(angle), this);
    }

    public void setArmAngle(double angle){
        armController.setReference(
            angle,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            armFeedforward.calculate(Units.degreesToRadians(angle),Units.degreesToRadians(armEncoder.getVelocity())));

    }
    
    public void periodic(){
        SmartDashboard.putNumber("Arm/ArmAngle", getArmAngle());
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
    }
}
