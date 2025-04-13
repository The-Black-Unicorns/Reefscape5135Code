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
package frc.robot.subsystems.arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.GenericPresicionSystemIO.Goal;

public class ArmSubsystem extends SubsystemBase{

    private double KP, KI, KD;
    private ArmSubsystemIO io;
    protected final ArmSubsystemIO.GenericPresicionSystemIOInputs inputs;


    private Goal goal;

    public ArmSubsystem(ArmSubsystemIO io) {
        this.io = io;

        this.inputs = new ArmSubsystemIO.GenericPresicionSystemIOInputs();
        goal = Goal.SCORE_MIDDLE;
        KP = Arm.ARM_KP;
        KI = Arm.ARM_KI;
        KD = Arm.ARM_KD;

    }

    public void setArmSpeed(double speed) {
        io.setSpeed(speed);
    }

    public void setArmVoltage(double voltage) {
        io.setVoltage(voltage);
    }

    public void stopArm() {
        io.stop();
    }

    public void updateInputs() {
        io.updateInputs(inputs);
    }

    public double getArmAngle() {
        return inputs.Angle;
    }

    public double getArmVelocity() {
        return inputs.Speed;
    }

    public double getArmTargetAngle() {
        return inputs.TargetAngle;
    }

    public double getArmMotorVoltageLeft() {
        return inputs.MotorVoltageLeft;
    }

    public double getArmMotorVoltageRight() {
        return inputs.MotorVoltageRight;
    }

    public double getArmMotorTempLeft() {
        return inputs.MotorTempLeft;
    }

    public double getArmMotorTempRight() {
        return inputs.MotorTempRight;
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

    public void armEnabledInit() {
        io.enabledInit();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Arm/ArmAngle", inputs.Angle);
        SmartDashboard.putNumber("Arm/armSpeed", inputs.Speed);
        SmartDashboard.putNumber("Arm/armVoltageR", inputs.MotorVoltageRight);
        SmartDashboard.putNumber("Arm/armVoltageL", inputs.MotorVoltageLeft);
        SmartDashboard.putNumber("Arm/armTempR", inputs.MotorTempRight);
        SmartDashboard.putNumber("Arm/armTempL", inputs.MotorTempLeft);
        SmartDashboard.putNumber("Arm/desiredAngle", inputs.TargetAngle);

        updateInputs();

        switch(goal){
            case CLIMB:
                setTargetAngle(Arm.ARM_CLIMB_ANGLE);
                break;
            case SCORE_MIDDLE:
                setTargetAngle(Arm.ARM_TOP_ANGLE);
                break;
            case INTAKE_DOWN:
                setTargetAngle(Arm.ARM_BOT_ANGLE);
                break;
            case INTAKE_UP:
                setTargetAngle(Arm.ARM_MID_ANGLE);
                break;
            case SCORE_UP:
                setTargetAngle(Arm.ARM_TOP_ANGLE);
                break;
        }
            

    }
    
    public void testPeriodic() {
        SmartDashboard.putNumber("Arm/armKp", SmartDashboard.getNumber("Arm/armKp", Arm.ARM_KP));
        SmartDashboard.putNumber("Arm/armKi", SmartDashboard.getNumber("Arm/armKi", Arm.ARM_KI));
        SmartDashboard.putNumber("Arm/armKd", SmartDashboard.getNumber("Arm/armKd", Arm.ARM_KD));
        double newKP = SmartDashboard.getNumber("Arm/armKp", Arm.ARM_KP);
        double newKI = SmartDashboard.getNumber("Arm/armKi", Arm.ARM_KI);
        double newKD = SmartDashboard.getNumber("Arm/armKd", Arm.ARM_KD);
        if(newKP != Arm.ARM_KP || newKI != Arm.ARM_KI || newKD != Arm.ARM_KD){
            KP = newKP;
            KI = newKI;
            KD = newKD;

            setPID(KP, KI, KD);
        }
    }

    public Command setTargetAngle(double position) {
        return runOnce(() ->io.setTargetAngle(position));
    }

    public Command moveArmTargetAngle() {
        return run(io::moveTargetAngle);
    }

    public Command setArmManualSpeed(double speed) {
        return run(() -> io.setSpeed(speed));
    }

    public Command setGoal(Goal goal) {
        return runOnce(() -> this.goal = goal);
    }

}
