package frc.robot.subsystems.swerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.io.File;
import java.util.function.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import static frc.robot.Constants.ControllerConstants.STICK_DEADBAND;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystem extends SubsystemBase {
    // public SwerveDriveOdometry swerveOdometry;
    // public SwerveModule[] mSwerveMods;
    // public AHRS gyro;
    // private PIDController aprilTagPIDController;
    private final SwerveDrive swerveDrive;

    private ChassisSpeeds currChassisSpeeds;
    private final PIDController xController = new PIDController(3, 0.1, 0.0);
    private final PIDController yController = new PIDController(3, 0.1, 0.0);
    private final PIDController headingController = new PIDController(1, 0.0, 0.0);

  Field2d field;
    public SwerveSubsystem(File directory) {
        // gyro = new AHRS(NavXComType.kMXP_SPI);
        field = new Field2d();
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED,
                                                                  new Pose2d(new Translation2d(1, 4),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1);

        // mSwerveMods = new SwerveModule[] {
        //         new SwerveModule(0, Constants.Swerve.Mod0.constants),
        //         new SwerveModule(1, Constants.Swerve.Mod1.constants),
        //         new SwerveModule(2, Constants.Swerve.Mod2.constants),
        //         new SwerveModule(3, Constants.Swerve.Mod3.constants)
        // };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        // currChassisSpeeds = new ChassisSpeeds();
        // swerveDrive.setModuleEncoderAutoSynchronize(true, 3);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // currChassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        //         translation.getX(),
        //         translation.getY(),
        //         rotation,
        //         getHeading())
        //         : new ChassisSpeeds(
        //                 translation.getX(),
        //                 translation.getY(),
        //                 rotation);

        // SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(currChassisSpeeds);
        // setModuleStates(swerveModuleStates);    
        System.out.println("translation: " + translation + " rotation: " + rotation);    
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);

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

    // public void driveForAuto(ChassisSpeeds chassisSpeeds) {
        
        
    //     SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
    //             .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroYaw()));
    //     setModuleStates(swerveModuleStates);
    // }

    /* Used by SwerveControllerCommand in Auto */
    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

    //     for (SwerveModule mod : mSwerveMods) {
    //         mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    //     }
    // }

    public SwerveModuleState[] getModuleStates() {
        return swerveDrive.getStates();

    }

    public SwerveModulePosition[] getModulePositions() {
        return swerveDrive.getModulePositions();
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    public void setPose(Pose2d pose) {
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
        swerveDrive.resetOdometry(pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
        //         new Pose2d(getPose().getTranslation(), heading));
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), heading));
    }


    public void zeroHeading() {
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
        //         new Pose2d(getPose().getTranslation(), new Rotation2d()));
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return swerveDrive.getYaw(); // changed from getyaw to fused heading
        //why is this (double)?
    }

    // public void resetModulesToAbsolute() {
    //     // for (SwerveModule mod : mSwerveMods) {
    //     //     mod.resetToAbsolute();
    //     // }
    //     swerveDrive.
    // }


   

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

    // private void setPIDRotation(double distanceFromAprilTagAngle){
    //     rotateRobot(aprilTagPIDController.calculate(distanceFromAprilTagAngle, 0));
    // }

    // public Command alignRobotToAprilTag(DoubleSupplier angleRelativeToAprilTag){
    //     return new RunCommand(() -> setPIDRotation(angleRelativeToAprilTag.getAsDouble()), this);
    // }
    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + xController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        swerveDrive.drive(speeds);
    }

    
//       StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
// .getStructArrayTopic("MyStates", SwerveModuleState.struct).publish();
//     StructPublisher<ChassisSpeeds> chpublisher = NetworkTableInstance.getDefault()
//     .getStructTopic("Speeds", getCurrentSpeeds().struct).publish();

    @Override
    public void periodic() {
    //  swerveOdometry.update(getGyroYaw(), getModulePositions());
    swervelib.SwerveModule[] mSwerveMods = swerveDrive.getModules();
      for (swervelib.SwerveModule mod : mSwerveMods) {
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getAbsoluteEncoder().getAbsolutePosition());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getAbsoluteEncoder().getAbsolutePosition());
          SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getAbsoluteEncoder().getAbsolutePosition());
      }
      swerveDrive.updateOdometry();
    //   System.out.println(gyro.getYaw());
        // publisher.set(getModuleStates());
        // chpublisher.set(getCurrentSpeeds());

      //System.out.println(getRobotOrientationForSpeaker());
      // System.out.println(mSwerveMods[4].getPosition());
      }    
    
}

