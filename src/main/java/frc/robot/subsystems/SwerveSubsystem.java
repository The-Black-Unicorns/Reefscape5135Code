package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.io.PrintStream;
import java.util.function.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants;

import static frc.robot.Constants.ControllerConstants.STICK_DEADBAND;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    private PIDController aprilTagPIDController;

    private ChassisSpeeds currChassisSpeeds;
    private final PIDController xController = new PIDController(3, 0.1, 0.0);
    private final PIDController yController = new PIDController(3, 0.1, 0.0);
    private final PIDController headingController = new PIDController(1, 0.0, 0.0);

    Field2d field;
    FieldObject2d wantedPoseFieldObject;

    StructPublisher<Pose2d> autonomousWantedPosePublisher;

    StructArrayPublisher<SwerveModuleState> swerveStatesPublisher;
    StructPublisher<ChassisSpeeds> chassisSpeedsPublisher;
    StructPublisher<Pose2d> currentPosePublisher; 

    NetworkTableInstance ntInstance;
  

    public SwerveSubsystem() {
        gyro = new AHRS(NavXComType.kMXP_SPI);
        field = new Field2d();
        wantedPoseFieldObject = field.getObject("Wanted Auto Pose");
        SmartDashboard.putData("field", field);

        ntInstance = NetworkTableInstance.getDefault();

        // General swerve information
        swerveStatesPublisher = ntInstance.getStructArrayTopic("Swerve State", SwerveModuleState.struct).publish();
        chassisSpeedsPublisher = ntInstance.getStructTopic("Current Chassis speeds", ChassisSpeeds.struct).publish();
        currentPosePublisher = ntInstance.getStructTopic("Current Pose", Pose2d.struct).publish();

        // Autonomous information
        autonomousWantedPosePublisher = ntInstance.getStructTopic("Autonomous Wanted Pose", Pose2d.struct).publish();

        
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        currChassisSpeeds = new ChassisSpeeds();
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        currChassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                rotation,
                getHeading())
                : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(currChassisSpeeds);
        setModuleStates(swerveModuleStates);        
    }

    public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angularSpeed,
            BooleanSupplier isFieldOriented) {
        return new RunCommand(() ->

        drive(
            new Translation2d(
                MathUtil.applyDeadband(xSpeed.getAsDouble(), STICK_DEADBAND),
                MathUtil.applyDeadband(ySpeed.getAsDouble(), STICK_DEADBAND)).times(MAX_SPEED),
            MathUtil.applyDeadband(angularSpeed.getAsDouble(), STICK_DEADBAND)  * MAX_ANGULAR_VELOCITY
            ,
            isFieldOriented.getAsBoolean(),
            true),

        this);
    }

    public Command driveConstantSpeed(double x, double y, double rotations, double time) {
        return new RunCommand(() -> drive(new Translation2d(x, y), rotations, true, true), this)
                .withTimeout(time)
                .andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, true)));
    }

    public void driveForAuto(ChassisSpeeds chassisSpeeds) {
        
        
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroYaw()));
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }


    public void zeroHeading() {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(-(double) gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return currChassisSpeeds;
    }

    public double getRobotOrientationForSpeaker(){
        double robotsOrientation = Math.signum(
            MathUtil.applyDeadband(getPose().getRotation().getDegrees(), 25));
        return robotsOrientation;
    }
    
    private void rotateRobot(double angularSpeed){
        drive(new Translation2d(
                MathUtil.applyDeadband(0, STICK_DEADBAND),
                MathUtil.applyDeadband(0, STICK_DEADBAND)).times(MAX_SPEED),
                MathUtil.applyDeadband(angularSpeed, STICK_DEADBAND) * MAX_ANGULAR_VELOCITY
                ,
                true,
                //!isFieldOriented.getAsBoolean(),
                true);
    }

    private void setPIDRotation(double distanceFromAprilTagAngle){
        rotateRobot(aprilTagPIDController.calculate(distanceFromAprilTagAngle, 0));
    }

    public Command alignRobotToAprilTag(DoubleSupplier angleRelativeToAprilTag){
        return new RunCommand(() -> setPIDRotation(angleRelativeToAprilTag.getAsDouble()), this);
    }


    public void followTrajectory(SwerveSample sample) {
        Pose2d currentRobotPose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(currentRobotPose.getX(), sample.x),
            sample.vy + yController.calculate(currentRobotPose.getY(), sample.y),
            sample.omega + xController.calculate(currentRobotPose.getRotation().getRadians(), sample.heading)
        );

        Pose2d wantedPose = new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading));

        currentPosePublisher.set(currentRobotPose);
        autonomousWantedPosePublisher.set(wantedPose);

        field.setRobotPose(currentRobotPose);
        wantedPoseFieldObject.setPose(wantedPose);

        driveForAuto(speeds);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());


        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        swerveStatesPublisher.set(getModuleStates());
        chassisSpeedsPublisher.set(getCurrentSpeeds());
        currentPosePublisher.set(getPose());
        
    }    
    
}

