// // // Copyright (c) FIRST and other WPILib contributors.
// // // Open Source Software; you can modify and/or share it under the terms of
// // // the WPILib BSD license file in the root directory of this project.

// // package frc.robot.subsystems;

// // import com.revrobotics.AbsoluteEncoder;
// // import com.revrobotics.spark.SparkMax;
// // import com.revrobotics.spark.SparkBase.PersistMode;
// // import com.revrobotics.spark.SparkBase.ResetMode;
// // import com.revrobotics.spark.SparkLowLevel.MotorType;
// // import com.revrobotics.spark.config.SparkMaxConfig;
// // import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// // import static frc.robot.Constants.PivotConstants.*;

// // import java.util.function.DoubleSupplier;

// // import edu.wpi.first.math.controller.ArmFeedforward;
// // import edu.wpi.first.math.controller.ProfiledPIDController;
// // import edu.wpi.first.math.trajectory.TrapezoidProfile;
// // import edu.wpi.first.wpilibj.RobotController;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// // import edu.wpi.first.wpilibj2.command.Command;
// // import edu.wpi.first.wpilibj2.command.RunCommand;
// // import edu.wpi.first.wpilibj2.command.SubsystemBase;
// // import frc.robot.Constants.Arm;

// // public class PivotSubsystem extends SubsystemBase {
// //   private final SparkMax pivotMotor;
// //   private final SparkMaxConfig pivotMotorConfig;
// //   private final AbsoluteEncoder pivotAbsoluteEncoder;

// //   private final ArmFeedforward pivotFeedforwardController;
// //   private final ProfiledPIDController pivotPIDController;

// //   private double currentPivotTargetAngle;

// //   double KP, KI, KD, KS, KV, KG;

// //   public PivotSubsystem(){
// //          pivotMotorConfig = new SparkMaxConfig();

// //          pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
         
// //          pivotMotorConfig.absoluteEncoder.positionConversionFactor(360);
// //          pivotMotorConfig.absoluteEncoder.velocityConversionFactor(6);

// //          pivotMotorConfig.inverted(PIVOT_MOTOR_INVERTED);
// //          pivotMotorConfig.idleMode(IdleMode.kBrake);
      
// //          pivotMotorConfig.absoluteEncoder.zeroOffset(PIVOT_ENCODER_OFFSET/360.0);
// //          pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         
// //          pivotFeedforwardController = new ArmFeedforward(Arm.ARM_KS,Arm.ARM_KG,Arm.ARM_KV);
// //          pivotAbsoluteEncoder = pivotMotor.getAbsoluteEncoder();
// //         pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, 
// //             new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
// //         pivotPIDController.setTolerance(Arm.ARM_POSITION_TOLERANCE_DEG);
// //         currentPivotTargetAngle = pivotAbsoluteEncoder.getPosition();
// //         pivotPIDController.enableContinuousInput(0, 360);

// //         KP = PIVOT_MOTOR_KP;
// //         KI = PIVOT_MOTOR_KI;
// //         KD = PIVOT_MOTOR_KD;

// //         KS = PIVOT_MOTOR_KS;
// //         KG = PIVOT_MOTOR_KG;
// //         KV = PIVOT_MOTOR_KV;

// //     }

// //     public TrapezoidProfile.State getCurrentState(){
// //       return new TrapezoidProfile.State(pivotAbsoluteEncoder.getPosition(), pivotAbsoluteEncoder.getVelocity());
// //   }

// //    public Command setDesiredPivotAngle() {
// //         return new RunCommand(() -> setPivotAngle(this::getDesiredPivotAngle), this);
// //     }


// //     public void setPivotAngle(DoubleSupplier targetAngleDegrees){
// //         System.out.println("angle: "+ targetAngleDegrees.getAsDouble());
// //         SmartDashboard.putNumber("Pivot/pivotDesiredAngle", targetAngleDegrees.getAsDouble());
// //         double ffVoltage = pivotFeedforwardController.calculate(pivotAbsoluteEncoder.getPosition()* Math.PI /
// //         180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
// //                                                     pivotAbsoluteEncoder.getVelocity()* Math.PI / 180.0);
// //         double pidVoltage = pivotPIDController.calculate(
// //           pivotAbsoluteEncoder.getPosition(), targetAngleDegrees.getAsDouble()
// //         );
        
// //         pivotMotor.setVoltage(pidVoltage + ffVoltage);

// //     }

// //   private double getPivotPosition(){
// //     return pivotAbsoluteEncoder.getPosition();
// //   }

// //   private double getPivotVelocity(){
// //     return pivotAbsoluteEncoder.getVelocity();
// //   }

// //   public void setPivotTargetAngle(double angleDeg){
// //     currentPivotTargetAngle = angleDeg;
// // }

// // public Command setDesiredAngleDeg(double desiredAngleDeg){
// //     return runOnce(()->setPivotTargetAngle(desiredAngleDeg));
// // }

// // public void pivotEnabledInit(){
// //   setPivotTargetAngle(pivotAbsoluteEncoder.getPosition());
// //   pivotPIDController.reset(pivotAbsoluteEncoder.getPosition());
// // }

// // public Command armEnabledInitCommand(){
// //   return runOnce(()->pivotEnabledInit());
// // }

// // private double getDesiredPivotAngle(){
// //   return currentPivotTargetAngle;
// // }

// //   @Override
// //   public void periodic() {
// //     double ffVoltage = pivotFeedforwardController.calculate(pivotAbsoluteEncoder.getPosition()* Math.PI /
// //     180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
// //                                                 pivotAbsoluteEncoder.getVelocity()* Math.PI / 180.0);
// //     double pidVoltage = pivotPIDController.calculate(
// //       pivotAbsoluteEncoder.getPosition(), 90);
    
// //     SmartDashboard.putNumber("Pivot/pivotAngle", getPivotPosition());
// //     SmartDashboard.putNumber("Pivot/pivotSpeed", getPivotVelocity());
// //     SmartDashboard.putNumber("Pivot/pivotPidVoltage", pidVoltage);
// //     SmartDashboard.putNumber("Pivot/pivotffVoltage", ffVoltage);
// //   }
  

  

// //   public void testPeriodic(){
// //     SmartDashboard.putNumber("Pivot/pivotKp", SmartDashboard.getNumber("Pivot/pivotKp", 0));
// //   SmartDashboard.putNumber("Pivot/pivotKi", SmartDashboard.getNumber("Pivot/pivotKi", 0));
// //   SmartDashboard.putNumber("Pivot/pivotKd", SmartDashboard.getNumber("Pivot/pivotKd", 0));
// //   double newKP = SmartDashboard.getNumber("Pivot/pivotKp", KP);
// //   double newKI = SmartDashboard.getNumber("Pivot/pivotKi", KI);
// //   double newKD = SmartDashboard.getNumber("Pivot/pivotKd", KD);
// //     if(newKP != KP || newKI != KI || newKD != KD){
// //       KP = newKP;
// //       KI = newKI;
// //       KD = newKD;
  
// //       pivotMotorConfig.closedLoop.pid(KP, KI, KD);
  
// //       pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
// //       }
// //     }

// //     public void setIdleModeBreak(){
// //       pivotMotorConfig.idleMode(IdleMode.kBrake);

// //       pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

// //   }
// //   public void setIdleModeCoast(){
// //       pivotMotorConfig.idleMode(IdleMode.kCoast);

// //       pivotMotor.configure(pivotMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

// //   }
// // }

// package  frc.robot.subsystems.pivot;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.Constants.PivotConstants;

// import static frc.robot.Constants.PivotConstants.*;

// import java.util.function.DoubleSupplier;

// public class PivotSubsystem extends SubsystemBase  {

    
//     private final SparkMax pivotMotor;
//     private AbsoluteEncoder pivotEncoder; 
//     // private SparkClosedLoopController armController;
//     private ProfiledPIDController pivotPIDController;
//     private ArmFeedforward pivotFeedforward;
//     private SparkMaxConfig pivotConfig;
//     private double currentPivotTargetAngle;
    
//     private double KP, KI, KD;
//     private double KS, KG, KV;

//     public PivotSubsystem(){
//          pivotConfig = new SparkMaxConfig();
         
//          pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

//          pivotConfig.absoluteEncoder.positionConversionFactor(360);
//          pivotConfig.absoluteEncoder.velocityConversionFactor(6);

//          pivotConfig.inverted(PIVOT_MOTOR_INVERTED);
//          pivotConfig.idleMode(IdleMode.kBrake);
//         //  armConfigR.closedLoop.
//         //  feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//         //  .pid(Arm.ARM_KP, Arm.ARM_KI, Arm.ARM_KD)
//         //  .maxMotion.maxVelocity(Arm.ARM_MAX_VELOCITY);
//         //  armConfigR.closedLoop.maxMotion.maxAcceleration(Arm.ARM_MAX_ACCELARATION);
      
//          pivotConfig.absoluteEncoder.zeroOffset(PIVOT_ENCODER_OFFSET/360.0);
//          pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
         
//         //  armController = armMotorR.getClosedLoopController();
//          pivotFeedforward = new ArmFeedforward(PivotConstants.PIVOT_MOTOR_KS,PivotConstants.PIVOT_MOTOR_KG,PivotConstants.PIVOT_MOTOR_KV);
//          pivotEncoder = pivotMotor.getAbsoluteEncoder();
//         pivotPIDController = new ProfiledPIDController(PIVOT_MOTOR_KP, PIVOT_MOTOR_KI, PIVOT_MOTOR_KD, 
//             new TrapezoidProfile.Constraints(MAX_PIVOT_DEGREES_PER_SECOND, MAX_PIVOT_DEGREES_PER_SECOND_SQUARED));
//         pivotPIDController.setTolerance(PIVOT_POSITION_TOLERANCE_DEG);
//         currentPivotTargetAngle = pivotEncoder.getPosition();
//         // pivotPIDController.enableContinuousInput(0, 360);

//         KP = PIVOT_MOTOR_KP;
//         KI = PIVOT_MOTOR_KI;
//         KD = PIVOT_MOTOR_KD;

//         KS = PIVOT_MOTOR_KS;
//         KG = PIVOT_MOTOR_KG;
//         KV = PIVOT_MOTOR_KV;

//     }

//     public TrapezoidProfile.State getCurrentState(){
//         return new TrapezoidProfile.State(pivotEncoder.getPosition(), pivotEncoder.getVelocity());
//     }

//     public void setIdleModeBreak(){
//         pivotConfig.idleMode(IdleMode.kBrake);

//         pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//     }
//     public void setIdleModeCoast(){
//         pivotConfig.idleMode(IdleMode.kCoast);

//         pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }
//     public double getPivotAngle(){
//         return pivotEncoder.getPosition();
//     }

//     public double getPivotVelocity(){
//         return pivotEncoder.getVelocity();
//     }

//     // public ProxyCommand controlArmMotorProxy(double angle){
//     //     return new ProxyCommand(new RunCommand(() -> setArmAngle(angle),this));
//     // }


//     public Command setDesiredAngle() {
//         return new RunCommand(() -> setPivotAngle(this::getDesiredPivotAngle), this);
//     }


//     public void setPivotAngle(DoubleSupplier targetAngleDegrees){


//         double ffVoltage = pivotFeedforward.calculate(pivotEncoder.getPosition()* Math.PI / 180.0 - PIVOT_NORMALIZE_OFFSET* Math.PI / 180.0,
//                                                     pivotEncoder.getVelocity()* Math.PI / 180.0);
//         double pidVoltage = pivotPIDController.calculate(pivotEncoder.getPosition(), targetAngleDegrees.getAsDouble()
//         );
        
//         pivotMotor.setVoltage(ffVoltage + pidVoltage);

//     }

//     public void setPivotManualSpeed(double speed){

//         // armMotorR.set(speed);
//         double ffVoltage = pivotFeedforward.calculate((pivotEncoder.getPosition() - PIVOT_NORMALIZE_OFFSET)* Math.PI / 180.0,
//         pivotEncoder.getVelocity()* Math.PI / 180.0);

//         pivotMotor.setVoltage(speed+ffVoltage);

//     }

//     public Command movePivotManulyCommand(DoubleSupplier speed){
//         return new RunCommand(() -> setPivotAngle(speed), this);
//     }

//     public void setPivotTargetAngle(double angleDeg){
//         currentPivotTargetAngle = angleDeg;
//     }

//     public Command setDesiredAngleDeg(double desiredAngleDeg){
//         return runOnce(()->setPivotTargetAngle(desiredAngleDeg));
//     }

//     public void pivotEnabledInit(){
//         setPivotTargetAngle(pivotEncoder.getPosition());
//         pivotPIDController.reset(pivotEncoder.getPosition());
//     }

//     public Command PivotEnabledInitCommand(){
//         return runOnce(()->pivotEnabledInit());
//     }

//     private double getDesiredPivotAngle(){
//         return currentPivotTargetAngle;
//     }

//     public Command setPivotAngleUpOuttake(){
//         return runOnce(()->setPivotTargetAngle(PIVOT_TOP_OUTTAKE_ANGLE));
//     }

//     public Command setPivotAngleUp(){
//         return runOnce(()->setPivotTargetAngle(PIVOT_TOP_ANGLE));
//     }

//     public Command setPivotAngleMiddle(){
//         return runOnce(()->setPivotTargetAngle(PIVOT_MID_ANGLE));
//     }

//     public Command setPivotAngleDown(){
//         return runOnce(()->setPivotTargetAngle(PIVOT_BOT_ANGLE));
//     }
    
    
    
//     @Override
//     public void periodic(){
//         double ffVoltage = pivotFeedforward.calculate(pivotEncoder.getPosition()* Math.PI / 180.0,
//         pivotEncoder.getVelocity()* Math.PI / 180.0);
//         // double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), 20.6);
//         // SmartDashboard.putNumber("Pivot/pidVoltage", pivotPIDController.calculate(pivotEncoder.getPosition(),90));
//         SmartDashboard.putNumber("Pivot/pivotAngle", getPivotAngle());
//         // SmartDashboard.putNumber("Arm/armSpeed", getArmVelocity());
//         // SmartDashboard.putNumber("Arm/armVoltageR", armMotorR.getBusVoltage());
//         // SmartDashboard.putNumber("Arm/armVoltageL", armMotorL.getBusVoltage());
//         SmartDashboard.putNumber("Pivot/desiredPivot", currentPivotTargetAngle);

//         // SmartDashboard.putNumber("Arm/ffVoltage", ffVoltage);
//         // SmartDashboard.putNumber("Arm/pidVoltage", pidVoltage);

//     }
//     public void testPeriodic(){
//         // double newKP = SmartDashboard.getNumber("Arm/armKp", KP);
//         // double newKI = SmartDashboard.getNumber("Arm/armKi", KI);
//         // double newKD = SmartDashboard.getNumber("Arm/armKd", KD);

//         currentPivotTargetAngle = SmartDashboard.getNumber("Pivot/desiredPivot", Constants.Arm.ARM_MID_ANGLE);
//         // if(newKP != KP || newKI != KI || newKD != KD){
//         //     KP = newKP;
//         //     KI = newKI;
//         //     KD = newKD;

//         //     pivotConfig.closedLoop.
//         //     feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//         //     .pid(newKP, newKI, newKD);

//         //     pivotConfig.closedLoop.
//         //     feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//         //     .pid(newKP, newKI, newKD);

//         //     pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
//         // }
//         // double newKS = SmartDashboard.getNumber("Arm/armKs", KS);
//         // double newKG = SmartDashboard.getNumber("Arm/armKg", KG);
//         // double newKV = SmartDashboard.getNumber("Arm/armKv", KV);

//         // if(newKS != KS || newKG != KG || newKV != KV){
//         //     KS = newKS;
//         //     KG = newKG;
//         //     KV = newKV;

//         //     pivotFeedforward = new ArmFeedforward(KS, KG, KV);
//         // }
//     }
// }

// package frc.robot.subsystems.pivot;

// package  frc.robot.subsystems.arm;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.AbsoluteEncoder;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.Constants;
// import frc.robot.Constants.Arm;

// import static frc.robot.Constants.Arm.*;

// import java.util.function.DoubleSupplier;

// public class ArmSubsystem extends SubsystemBase  {

    
//     private final SparkMax armMotorR,armMotorL;
//     private AbsoluteEncoder armEncoder; 
//     // private SparkClosedLoopController armController;
//     private ProfiledPIDController armPIDController;
//     private ArmFeedforward armFeedforward;
//     private SparkMaxConfig armConfigL,armConfigR;
//     private double currentArmTargetAngle;
    
//     private double KP, KI, KD;
//     private double KS, KG, KV;

//     public ArmSubsystem(){
//          armConfigL = new SparkMaxConfig();
//          armConfigR = new SparkMaxConfig();
         
//          armMotorR = new SparkMax(RIGHT_ARM_MOTOR, MotorType.kBrushless);
//          armMotorL = new SparkMax(LEFT_ARM_MOTOR, MotorType.kBrushless);
         
//          armConfigR.absoluteEncoder.positionConversionFactor(360);
//          armConfigR.absoluteEncoder.velocityConversionFactor(6);

//          armConfigR.inverted(false);
//          armConfigR.idleMode(IdleMode.kBrake);
//         //  armConfigR.closedLoop.
//         //  feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//         //  .pid(Arm.ARM_KP, Arm.ARM_KI, Arm.ARM_KD)
//         //  .maxMotion.maxVelocity(Arm.ARM_MAX_VELOCITY);
//         //  armConfigR.closedLoop.maxMotion.maxAcceleration(Arm.ARM_MAX_ACCELARATION);
//          armConfigL.apply(armConfigR);
//          armConfigR.follow(armMotorL, true);
//          armConfigL.absoluteEncoder.zeroOffset(Arm.ARM_ENCODER_OFFSET/360.0);
//          armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//          armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         //  armController = armMotorR.getClosedLoopController();
//          armFeedforward = new ArmFeedforward(Arm.ARM_KS,Arm.ARM_KG,Arm.ARM_KV);
//          armEncoder = armMotorL.getAbsoluteEncoder();
//         armPIDController = new ProfiledPIDController(ARM_KP, ARM_KI, ARM_KD, 
//             new TrapezoidProfile.Constraints(ARM_MAX_VELOCITY, ARM_MAX_ACCELARATION));
//         armPIDController.setTolerance(Arm.ARM_POSITION_TOLERANCE_DEG);
//         currentArmTargetAngle = armEncoder.getPosition();
//         armPIDController.enableContinuousInput(0, 360);

//         KP = Arm.ARM_KP;
//         KI = Arm.ARM_KI;
//         KD = Arm.ARM_KD;

//         KS = Arm.ARM_KS;
//         KG = Arm.ARM_KG;
//         KV = Arm.ARM_KV;

//     }

//     public TrapezoidProfile.State getCurrentState(){
//         return new TrapezoidProfile.State(armEncoder.getPosition(), armEncoder.getVelocity());
//     }

//     public void setIdleModeBreak(){
//         armConfigR.idleMode(IdleMode.kBrake);
//         armConfigL.idleMode(IdleMode.kBrake);
//         armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//         armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

//     }
//     public void setIdleModeCoast(){
//         armConfigR.idleMode(IdleMode.kCoast);
//         armConfigL.idleMode(IdleMode.kCoast);
//         armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         System.out.println("set coast");
//     }
//     public double getArmAngle(){
//         return armEncoder.getPosition();
//     }

//     public double getArmVelocity(){
//         return armEncoder.getVelocity();
//     }

//     // public ProxyCommand controlArmMotorProxy(double angle){
//     //     return new ProxyCommand(new RunCommand(() -> setArmAngle(angle),this));
//     // }


//     public Command setDesiredAngle() {
//         return new RunCommand(() -> setArmAngle(this::getDesiredArmAngle), this);
//     }


//     public void setArmAngle(DoubleSupplier targetAngleDegrees){


//         double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0 - ARM_NORMALIZE_OFFSET* Math.PI / 180.0,
//                                                     armEncoder.getVelocity()* Math.PI / 180.0);
//         double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), targetAngleDegrees.getAsDouble()
//         );
        
//         armMotorL.setVoltage(ffVoltage + pidVoltage);

//     }

//     public void setArmManualSpeed(double speed){

//         // armMotorR.set(speed);
//         double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0,
//         armEncoder.getVelocity()* Math.PI / 180.0);

//         armMotorL.setVoltage(speed+ffVoltage);

//     }

//     public Command moveArmManulyCommand(DoubleSupplier speed){
//         return new RunCommand(() -> setArmAngle(speed), this);
//     }

//     public void setArmTargetAngle(double angleDeg){
//         currentArmTargetAngle = angleDeg;
//     }

//     public Command setDesiredAngleDeg(double desiredAngleDeg){
//         return runOnce(()->setArmTargetAngle(desiredAngleDeg));
//     }

//     public Command setArmAngleToClimb(){
//         return runOnce(()->setArmTargetAngle(ARM_CLIMB_ANGLE));
//     }

//     public Command setArmAngleUp(){
//         return runOnce(()->setArmTargetAngle(ARM_TOP_ANGLE));
//     }

//     public Command setArmAngleMiddle(){
//         return runOnce(()->setArmTargetAngle(ARM_MID_ANGLE));
//     }

//     public Command setArmAngleDown(){
//         return runOnce(()->setArmTargetAngle(ARM_BOT_ANGLE));
//     }

//     public void armEnabledInit(){
//         setArmTargetAngle(armEncoder.getPosition());
//         armPIDController.reset(armEncoder.getPosition());
//     }

//     public Command armEnabledInitCommand(){
//         return runOnce(()->armEnabledInit());
//     }

//     private double getDesiredArmAngle(){
//         return currentArmTargetAngle;
//     }

    
    
    

//     public void periodic(){
//         double ffVoltage = armFeedforward.calculate(armEncoder.getPosition()* Math.PI / 180.0,
//         armEncoder.getVelocity()* Math.PI / 180.0);
//         // double pidVoltage = armPIDController.calculate(armEncoder.getPosition(), 20.6);
//         // SmartDashboard.putNumber("Arm/pidVoltage", armPIDController.calculate(armEncoder.getPosition(), 60));
//         SmartDashboard.putNumber("Arm/ArmAngle", getArmAngle());
//         SmartDashboard.putNumber("Arm/armSpeed", getArmVelocity());
//         SmartDashboard.putNumber("Arm/armVoltageR", armMotorR.getBusVoltage());
//         SmartDashboard.putNumber("Arm/armVoltageL", armMotorL.getBusVoltage());
//         SmartDashboard.putNumber("Arm/desiredAngle", currentArmTargetAngle);

//         SmartDashboard.putNumber("Arm/ffVoltage", ffVoltage);
//         // SmartDashboard.putNumber("Arm/pidVoltage", pidVoltage);

//     }
//     public void testPeriodic(){
//         SmartDashboard.putNumber("Arm/armKp", SmartDashboard.getNumber("Arm/armKp", 0.15));
//         SmartDashboard.putNumber("Arm/armKi", SmartDashboard.getNumber("Arm/armKi", 0.01));
//         SmartDashboard.putNumber("Arm/armKd", SmartDashboard.getNumber("Arm/armKd", 0.04));

        
//         double newKP = SmartDashboard.getNumber("Arm/armKp", KP);
//         double newKI = SmartDashboard.getNumber("Arm/armKi", KI);
//         double newKD = SmartDashboard.getNumber("Arm/armKd", KD);

//         currentArmTargetAngle = SmartDashboard.getNumber("Arm/desiredAngle", Constants.Arm.ARM_MID_ANGLE);

//         if(newKP != KP || newKI != KI || newKD != KD){
//             KP = newKP;
//             KI = newKI;
//             KD = newKD;

//             armConfigR.closedLoop.
//             feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//             .pid(newKP, newKI, newKD);

//             armConfigL.closedLoop.
//             feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//             .pid(newKP, newKI, newKD);

//             armMotorR.configure(armConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//             armMotorL.configure(armConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//         }
//         double newKS = SmartDashboard.getNumber("Arm/armKs", KS);
//         double newKG = SmartDashboard.getNumber("Arm/armKg", KG);
//         double newKV = SmartDashboard.getNumber("Arm/armKv", KV);

//         if(newKS != KS || newKG != KG || newKV != KV){
//             KS = newKS;
//             KG = newKG;
//             KV = newKV;

//             armFeedforward = new ArmFeedforward(KS, KG, KV);
//         }
//     }

    
// }


package frc.robot.subsystems.pivot;


import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KD;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KI;
import static frc.robot.Constants.PivotConstants.PIVOT_MOTOR_KP;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.GenericPresicionSystemIO.Goal;;

public class PivotSubsystem extends SubsystemBase{

    private double KP, KI, KD;
    private PivotSubsystemIO io;
    protected final PivotSubsystemIO.GenericPresicionSystemIOInputs inputs;

    private Goal goal;

    public PivotSubsystem(PivotSubsystemIO io) {
        this.io = io;

        this.inputs = new PivotSubsystemIO.GenericPresicionSystemIOInputs();
        goal = Goal.IDLE;
        KP = PIVOT_MOTOR_KP;
        KI = PIVOT_MOTOR_KI;
        KD = PIVOT_MOTOR_KD;

    }

    public void setPivotSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setPivotVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stopPivot() {
        io.stop();
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public double getPivotAngle() {
        return inputs.Angle;
    }

    public double getVelocity() {
        return inputs.Speed;
    }

    public double getTargetAngle() {
        return inputs.TargetAngle;
    }

    public double getMotorVoltage() {
        return inputs.MotorVoltageLeft;
    }

    public double getMotorTemp() {
        return inputs.MotorTempLeft;
    }

    public void setPID(double KP, double KI, double KD) {
        io.setPID(KP, KI, KD);
    }

    public void setPIDF(double KP, double KI, double KD, double KS, double KG, double KV) {
        io.setPIDF(KP, KI, KD, KS, KG, KV);
    }

    public Command setIdleModeBreak() {
        return runOnce(io::setIdleModeBreak);
    }

    public Command setIdleModeCoast() {
        return runOnce(io::setIdleModeCoast);
    }

    public void pivotEnabledInit() {
        io.enabledInit();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Pivot/PivotAngle", inputs.Angle);

        SmartDashboard.putNumber("Pivot/PivotVoltage", inputs.MotorVoltageLeft);
        SmartDashboard.putNumber("Pivot/PivotTemp", inputs.MotorTempLeft);
        SmartDashboard.putNumber("Pivot/desiredAngle", inputs.TargetAngle);

        updateInputs();

        switch(goal){
            case CLIMB:
                io.setTargetAngle(PivotConstants.PIVOT_BOT_ANGLE);
                break;
            case SCORE_MIDDLE:
                io.setTargetAngle(PivotConstants.PIVOT_MID_ANGLE);
                break;
            case INTAKE_DOWN:
                io.setTargetAngle(PivotConstants.PIVOT_BOT_ANGLE);
                break;
            case INTAKE_UP:
                io.setTargetAngle(PivotConstants.PIVOT_TOP_ANGLE);
                break;
            case SCORE_UP:
                io.setTargetAngle(PivotConstants.PIVOT_TOP_ANGLE);
                break;
            case IDLE:
                io.setTargetAngle(inputs.TargetAngle);
                break;
            default:
                io.setTargetAngle(inputs.TargetAngle);
                break;
        }
    // System.out.println(goal.name());
            

    }
    
    public void testPeriodic() {
        SmartDashboard.putNumber("Pivot/pivotKp", SmartDashboard.getNumber("Pivot/pivotKp", PIVOT_MOTOR_KP));
        SmartDashboard.putNumber("Pivot/pivotKi", SmartDashboard.getNumber("Pivot/pivotKi", PIVOT_MOTOR_KI));
        SmartDashboard.putNumber("Pivot/pivotKd", SmartDashboard.getNumber("Pivot/pivotKd", PIVOT_MOTOR_KD));
        double newKP = SmartDashboard.getNumber("Pivot/pivotKp", PIVOT_MOTOR_KP);
        double newKI = SmartDashboard.getNumber("Pivot/pivotKi", PIVOT_MOTOR_KI);
        double newKD = SmartDashboard.getNumber("Pivot/pivotKd", PIVOT_MOTOR_KD);
        if (newKP != PIVOT_MOTOR_KP || newKI != PIVOT_MOTOR_KI || newKD != PIVOT_MOTOR_KD) {
            KP = newKP;
            KI = newKI;
            KD = newKD;

            setPID(KP, KI, KD);
        }
    }

    public Command setTargetAngle(double position) {
        return runOnce(() ->io.setTargetAngle(position));
    }

    public Command movePivotTargetAngle() {
        return run(io::moveTargetAngle);
    }

    public Command setPivotManualSpeed(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    public Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

}




