// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// import static frc.robot.Constants.PivotConstants.*;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.Arm;

// public class PivotSubsystem extends SubsystemBase {
//   private final SparkMax pivotMotor;
//   private final SparkMaxConfig pivotMotorConfig;
//   private final AbsoluteEncoder pivotAbsoluteEncoder;

//   private final ArmFeedforward pivotFeedforwardController;
//   private final ProfiledPIDController pivotPIDController;

//   private double currentPivotTargetAngle;

//   double KP, KI, KD, KS, KV, KG;

//   public PivotSubsystem(){
//          pivotMotorConfig = new SparkMaxConfig();

//          pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
         
//          pivotMotorConfig.absoluteEncoder.positionConversionFactor(360);
//          pivotMotorConfig.absoluteEncoder.velocityConversionFactor(6);

//          pivotMotorConfig.inverted(PIVOT_MOTOR_INVERTED);
//          pivotMotorConfig.idleMode(IdleMode.kBrake);
      
//          pivotMotorConfig.absoluteEncoder.zeroOffset(PIVOT_ENCODER_OFFSET/360.0);
//          pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         
//          pivotFeedforwardController = new ArmFeedforward(Arm.ARM_KS,Arm.ARM_KG,Arm.ARM_KV);
//          pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
//         pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, 
//             new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
//         pivotPIDController.setTolerance(Arm.ARM_POSITION_TOLERANCE_DEG);
//         currentPivotTargetAngle = pivotAbsoluteEncoder.getPosition();
//         pivotPIDController.enableContinuousInput(0, 360);

//         KP = PIVOT_MOTOR_KP;
//         KI = PIVOT_MOTOR_KI;
//         KD = PIVOT_MOTOR_KD;

//         KS = PIVOT_MOTOR_KS;
//         KG = PIVOT_MOTOR_KG;
//         KV = PIVOT_MOTOR_KV;

//     }

//     public TrapezoidProfile.State getCurrentState(){
//       return new TrapezoidProfile.State(pivotAbsoluteEncoder.getPosition(), pivotAbsoluteEncoder.getVelocity());
//   }

//    public Command setDesiredPivotAngle() {
//         return new RunCommand(() -> setPivotAngle(this::getDesiredPivotAngle), this);
//     }


//     public void setPivotAngle(DoubleSupplier targetAngleDegrees){
//         System.out.println("angle: "+ targetAngleDegrees.getAsDouble());
//         SmartDashboard.putNumber("Pivot/pivotDesiredAngle", targetAngleDegrees.getAsDouble());
//         double ffVoltage = pivotFeedforwardController.calculate(pivotAbsoluteEncoder.getPosition()* Math.PI /
//         180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
//                                                     pivotAbsoluteEncoder.getVelocity()* Math.PI / 180.0);
//         double pidVoltage = pivotPIDController.calculate(
//           pivotAbsoluteEncoder.getPosition(), targetAngleDegrees.getAsDouble()
//         );
        
//         pivotMotor.setVoltage(pidVoltage + ffVoltage);

//     }

//   private double getPivotPosition(){
//     return pivotAbsoluteEncoder.getPosition();
//   }

//   private double getPivotVelocity(){
//     return pivotAbsoluteEncoder.getVelocity();
//   }

//   public void setPivotTargetAngle(double angleDeg){
//     currentPivotTargetAngle = angleDeg;
// }

// public Command setDesiredAngleDeg(double desiredAngleDeg){
//     return runOnce(()->setPivotTargetAngle(desiredAngleDeg));
// }

// public void pivotEnabledInit(){
//   setPivotTargetAngle(pivotAbsoluteEncoder.getPosition());
//   pivotPIDController.reset(pivotAbsoluteEncoder.getPosition());
// }

// public Command armEnabledInitCommand(){
//   return runOnce(()->pivotEnabledInit());
// }

// private double getDesiredPivotAngle(){
//   return currentPivotTargetAngle;
// }

//   @Override
//   public void periodic() {
//     double ffVoltage = pivotFeedforwardController.calculate(pivotAbsoluteEncoder.getPosition()* Math.PI /
//     180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
//                                                 pivotAbsoluteEncoder.getVelocity()* Math.PI / 180.0);
//     double pidVoltage = pivotPIDController.calculate(
//       pivotAbsoluteEncoder.getPosition(), 90);
    
//     SmartDashboard.putNumber("Pivot/pivotAngle", getPivotPosition());
//     SmartDashboard.putNumber("Pivot/pivotSpeed", getPivotVelocity());
//     SmartDashboard.putNumber("Pivot/pivotPidVoltage", pidVoltage);
//     SmartDashboard.putNumber("Pivot/pivotffVoltage", ffVoltage);
//   }
  

  

//   public void testPeriodic(){
//     SmartDashboard.putNumber("Pivot/pivotKp", SmartDashboard.getNumber("Pivot/pivotKp", 0));
//   SmartDashboard.putNumber("Pivot/pivotKi", SmartDashboard.getNumber("Pivot/pivotKi", 0));
//   SmartDashboard.putNumber("Pivot/pivotKd", SmartDashboard.getNumber("Pivot/pivotKd", 0));
//   double newKP = SmartDashboard.getNumber("Pivot/pivotKp", KP);
//   double newKI = SmartDashboard.getNumber("Pivot/pivotKi", KI);
//   double newKD = SmartDashboard.getNumber("Pivot/pivotKd", KD);
//     if(newKP != KP || newKI != KI || newKD != KD){
//       KP = newKP;
//       KI = newKI;
//       KD = newKD;
  
//       pivotMotorConfig.closedLoop.pid(KP, KI, KD);
  
//       pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//       }
//     }

//     public void setIdleModeBreak(){
//       pivotMotorConfig.idleMode(IdleMode.kBrake);

//       pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//   }
//   public void setIdleModeCoast(){
//       pivotMotorConfig.idleMode(IdleMode.kCoast);

//       pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//   }
// }

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
import frc.robot.Constants.PivotConstants;

import static frc.robot.Constants.PivotConstants.*;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase  {

    
    private final SparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder; 
    // private SparkClosedLoopController armController;
    private ProfiledPIDController pivotPIDController;
    private ArmFeedforward pivotFeedforward;
    private SparkMaxConfig pivotConfig;
    private double currentPivotTargetAngle;
    
    private double KP, KI, KD;
    private double KS, KG, KV;

    public PivotSubsystem(){
         pivotConfig = new SparkMaxConfig();
         
         pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

         pivotConfig.absoluteEncoder.positionConversionFactor(360);
         pivotConfig.absoluteEncoder.velocityConversionFactor(6);

         pivotConfig.inverted(PIVOT_MOTOR_INVERTED);
         pivotConfig.idleMode(IdleMode.kBrake);
        //  armConfigR.closedLoop.
        //  feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        //  .pid(Arm.ARM_KP, Arm.ARM_KI, Arm.ARM_KD)
        //  .maxMotion.maxVelocity(Arm.ARM_MAX_VELOCITY);
        //  armConfigR.closedLoop.maxMotion.maxAcceleration(Arm.ARM_MAX_ACCELARATION);
      
         pivotConfig.absoluteEncoder.zeroOffset(PIVOT_ENCODER_OFFSET/360.0);
         pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         
        //  armController = armMotorR.getClosedLoopController();
         pivotFeedforward = new ArmFeedforward(PivotConstants.PIVOT_MOTOR_KS,PivotConstants.PIVOT_MOTOR_KG,PivotConstants.PIVOT_MOTOR_KV);
         pivotEncoder = pivotMotor.getAbsoluteEncoder();
        pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, 
            new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
        pivotPIDController.setTolerance(PIVOT_POSITION_TOLERANCE_DEG);
        currentPivotTargetAngle = pivotEncoder.getPosition();
        // pivotPIDController.enableContinuousInput(0, 360);

        KP = PIVOT_MOTOR_KP;
        KI = PIVOT_MOTOR_KI;
        KD = PIVOT_MOTOR_KD;

        KS = PIVOT_MOTOR_KS;
        KG = PIVOT_MOTOR_KG;
        KV = PIVOT_MOTOR_KV;

    }

    public TrapezoidProfile.State getCurrentState(){
        return new TrapezoidProfile.State(pivotEncoder.getPosition(), pivotEncoder.getVelocity());
    }

    public void setIdleModeBreak(){
        pivotConfig.idleMode(IdleMode.kBrake);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }
    public void setIdleModeCoast(){
        pivotConfig.idleMode(IdleMode.kCoast);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public double getPivotAngle(){
        return pivotEncoder.getPosition();
    }

    public double getPivotVelocity(){
        return pivotEncoder.getVelocity();
    }

    // public ProxyCommand controlArmMotorProxy(double angle){
    //     return new ProxyCommand(new RunCommand(() -> setArmAngle(angle),this));
    // }


    public Command setDesiredAngle() {
        return new RunCommand(() -> setPivotAngle(this::getDesiredPivotAngle), this);
    }


    public void setPivotAngle(DoubleSupplier targetAngleDegrees){


        double ffVoltage = pivotFeedforward.calculate(pivotEncoder.getPosition()* Math.PI / 180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
                                                    pivotEncoder.getVelocity()* Math.PI / 180.0);
        double pidVoltage = pivotPIDController.calculate(pivotEncoder.getPosition(), targetAngleDegrees.getAsDouble()
        );
        
        pivotMotor.setVoltage(ffVoltage + pidVoltage);

    }

    public void setPivotManualSpeed(double speed){

        // armMotorR.set(speed);
        double ffVoltage = pivotFeedforward.calculate((pivotEncoder.getPosition() - PIVOT_NORMALIZE_OFFSET)* Math.PI / 180.0,
        pivotEncoder.getVelocity()* Math.PI / 180.0);

        pivotMotor.setVoltage(speed+ffVoltage);

    }

    public Command movePivotManulyCommand(DoubleSupplier speed){
        return new RunCommand(() -> setPivotAngle(speed), this);
    }

    public void setPivotTargetAngle(double angleDeg){
        currentPivotTargetAngle = angleDeg;
    }

    public Command setDesiredAngleDeg(double desiredAngleDeg){
        return runOnce(()->setPivotTargetAngle(desiredAngleDeg));
    }

    public void pivotEnabledInit(){
        setPivotTargetAngle(pivotEncoder.getPosition());
        pivotPIDController.reset(pivotEncoder.getPosition());
    }

    public Command PivotEnabledInitCommand(){
        return runOnce(()->pivotEnabledInit());
    }

    private double getDesiredPivotAngle(){
        return currentPivotTargetAngle;
    }
    
    
    

    public void periodic(){
        double ffVoltage = pivotFeedforward.calculate(pivotEncoder.getPosition()* Math.PI / 180.0,
        pivotEncoder.getVelocity()* Math.PI / 180.0);
        // double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), 20.6);
        SmartDashboard.putNumber("Pivot/pidVoltage", pivotPIDController.calculate(pivotEncoder.getPosition(),90));
        SmartDashboard.putNumber("Pivot/pivotAngle", getPivotAngle());
        // SmartDashboard.putNumber("Arm/armSpeed", getArmVelocity());
        // SmartDashboard.putNumber("Arm/armVoltageR", armMotorR.getBusVoltage());
        // SmartDashboard.putNumber("Arm/armVoltageL", armMotorL.getBusVoltage());
        SmartDashboard.putNumber("Pivot/desiredPivot", currentPivotTargetAngle);

        // SmartDashboard.putNumber("Arm/ffVoltage", ffVoltage);
        // SmartDashboard.putNumber("Arm/pidVoltage", pidVoltage);

    }
    public void testPeriodic(){
        // double newKP = SmartDashboard.getNumber("Arm/armKp", KP);
        // double newKI = SmartDashboard.getNumber("Arm/armKi", KI);
        // double newKD = SmartDashboard.getNumber("Arm/armKd", KD);

        // if(newKP != KP || newKI != KI || newKD != KD){
        //     KP = newKP;
        //     KI = newKI;
        //     KD = newKD;

        //     pivotConfig.closedLoop.
        //     feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        //     .pid(newKP, newKI, newKD);

        //     pivotConfig.closedLoop.
        //     feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        //     .pid(newKP, newKI, newKD);

        //     pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
        // }
        // double newKS = SmartDashboard.getNumber("Arm/armKs", KS);
        // double newKG = SmartDashboard.getNumber("Arm/armKg", KG);
        // double newKV = SmartDashboard.getNumber("Arm/armKv", KV);

        // if(newKS != KS || newKG != KG || newKV != KV){
        //     KS = newKS;
        //     KG = newKG;
        //     KV = newKV;

        //     pivotFeedforward = new ArmFeedforward(KS, KG, KV);
        // }
    }
}

