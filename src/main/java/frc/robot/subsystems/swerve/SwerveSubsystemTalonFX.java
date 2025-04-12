package frc.robot.subsystems.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.io.File;
import java.util.function.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import static frc.robot.Constants.ControllerConstants.STICK_DEADBAND;
import static frc.robot.Constants.Swerve.*;

public class SwerveSubsystemTalonFX /*extends SubsystemBase*/ implements SwerveSubsystemIO {
    // public SwerveDriveOdometry swerveOdometry;
    // public SwerveModule[] mSwerveMods;
    // public AHRS gyro;
    // private PIDController aprilTagPIDController;
    private final SwerveDrive swerveDrive;

    // private ChassisSpeeds currChassisSpeeds;
    private final PIDController xController = new PIDController(3, 0, 0.0);
    private final PIDController yController = new PIDController(3 , 0, 0.0);
    private final PIDController headingController = new PIDController(3, 0.0, 0.0);
    private boolean doRejectUpdate;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(10);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(10);

    public SwerveSubsystemTalonFX(File directory) {

        // gyro = new AHRS(NavXComType.kMXP_SPI);


        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED,
                                                                  new Pose2d(new Translation2d(15, 4),
                                                                             Rotation2d.fromDegrees(0)));
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    

    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(false,
                                               false,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    // swerveDrive.setModuleEncoderAutoSynchronize(true,
                                                // 3); //try to use this today

        // mSwerveMods = new SwerveModule[] {
        //         new SwerveModule(0, Constants.Swerve.Mod0.constants),
        //         new SwerveModule(1, Constants.Swerve.Mod1.constants),
        //         new SwerveModule(2, Constants.Swerve.Mod2.constants),
        //         new SwerveModule(3, Constants.Swerve.Mod3.constants)
        // };

        // swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        // currChassisSpeeds = new ChassisSpeeds();
        // swerveDrive.setModuleEncoderAutoSynchronize(true, 3);
        swerveDrive.synchronizeModuleEncoders();
        swerveDrive.pushOffsetsToEncoders();
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        doRejectUpdate = false;

        //disable this if no vision
        swerveDrive.stopOdometryThread();


        
    }

    @Override
    public void updateOdometry() {
        
        swerveDrive.updateOdometry();
    }


    // @Override
    // public void periodic() {
    //     // System.out.println(isRedAlliance());
    // //  swerveOdometry.update(getGyroYaw(), getModulePositions());
    // // swervelib.SwerveModule[] mSwerveMods = swerveDrive.getModules();
    // //   for (swervelib.SwerveModule mod : mSwerveMods) {

    // //       SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getAbsolutePosition());
    // //       SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getAbsoluteEncoder().getAbsolutePosition());
    // //       SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getAbsoluteEncoder().getAbsolutePosition());
    // //   }

    // curPosePublisher.set(getPose());
    
    // swerveDrive.updateOdometry();
    //   updateLimelightReading(() -> swerveDrive.getPose().getRotation().getDegrees(),()->  (swerveDrive.getFieldVelocity().omegaRadiansPerSecond * 180.0 / Math.PI));
    // //   System.out.println(gyro.getYaw());
    //     // publisher.set(getModuleStates());
    //     // chpublisher.set(getCurrentSpeeds());

    //   //System.out.println(getRobotOrientationForSpeaker());
    //   // System.out.println(mSwerveMods[4].getPosition());
    // //   System.out.println(swerveDrive.getPose());
    // //   System.out.println(swerveDrive.getYaw());
    //   }    

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // Translation2d lastSpeeds = 
        // new Translation2d(swerveDrive.getFieldVelocity().vxMetersPerSecond, swerveDrive.getFieldVelocity().vyMetersPerSecond);
        // Translation2d newTranslation = SwerveMath.normalizeWheelAccel(translation, lastSpeeds);

        // double x_speed = translation.getX();
        // double y_speed = translation.getY();
        // double x = xLimiter.calculate(x_speed);
        // double y = yLimiter.calculate(y_speed);
        // translation = new Translation2d(x, y);

        int invertInputs = isRedAlliance() ? -1 : 1;
        swerveDrive.drive(translation.times(invertInputs), rotation, fieldRelative, isOpenLoop);
        // swerveDrive.driveFieldOriented(getCurrentSpeeds());
    }

    // public Command driveCommandForDriver(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier angularSpeed,
    //         BooleanSupplier isFieldOriented, DoubleSupplier speedExponent) {
    //     // int invertInputs = isRedAlliance() ? -1 : 1;

    //     return new RunCommand(() ->

    //     drive(
    //         new Translation2d(
    //             MathUtil.applyDeadband(xSpeed.getAsDouble(), STICK_DEADBAND),
    //             MathUtil.applyDeadband(ySpeed.getAsDouble(), STICK_DEADBAND)).times(MAX_SPEED).times(speedExponent.getAsDouble() * 
    //             Math.abs(speedExponent.getAsDouble())),
    //         MathUtil.applyDeadband(angularSpeed.getAsDouble(), STICK_DEADBAND)  * MAX_ANGULAR_VELOCITY
    //         ,
    //         isFieldOriented.getAsBoolean(),
    //         true),//check if closed loop is better then open loop

    //     this);
    // }

    // public Command driveConstantSpeed(double x, double y, double rotations, double time, boolean isFieldOriented) {
    //     return new RunCommand(() -> drive(new Translation2d(x, y), rotations, isFieldOriented, true), this)
    //             .withTimeout(time)
    //             .andThen(new InstantCommand(() -> drive(new Translation2d(), 0, true, true)));
    // }

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

    //Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
    //Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // doesnt set the gyro yaw only the heading!!!
    public void setHeading(Rotation2d heading) {
        // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
        //         new Pose2d(getPose().getTranslation(), heading));
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), heading));
    }

    // makes robot face towards 0, red alliance wall
    public void zeroGyro() {

        swerveDrive.zeroGyro();
    }

    public void zeroGyroWithAlliance()
    {
    //   if (!
    //   isRedAlliance())
    //   {
    //     zeroGyro();
    //     //Set the pose 180 degrees
    //     swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
        
    //   } else
    //   {
    if(!isRedAlliance()){
        zeroGyro();
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else{
        zeroGyro();
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    }
    //   }
    }

    public void zeroGyroAutonomous()
    {
    //   if (!
    //   isRedAlliance())
    //   {
        zeroGyro();
        //Set the pose 180 degrees
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
        
    //   } else
    //   {
    //     zeroGyro();
    //     swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    //   }
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
        return swerveDrive.getFieldVelocity();
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
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        
        



        // Apply the generated speeds
        swerveDrive.driveFieldOriented(speeds);
    }

    // public double getLimelightMegatag1AngleDeg(){
        
    // }

    public void updateLimelightReading(DoubleSupplier robotYaw, DoubleSupplier robotYawRate){
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_NAME, robotYaw.getAsDouble(), robotYawRate.getAsDouble(), 0,0,0,0);
        LimelightHelpers.PoseEstimate mt2;
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        // LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);
        doRejectUpdate = false;
        if(Math.abs(robotYawRate.getAsDouble()) > 720) doRejectUpdate = true;
        else if(mt2 == null) doRejectUpdate = true;
        else if(mt2.tagCount == 0){ 
            doRejectUpdate = true;
            // System.out.println("found 0 tags");
        }
        // if(mt2 == null) System.out.println("bad");

        if(!doRejectUpdate){
            swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,  VecBuilder.fill(0.3,0.3,9999999));
            // System.out.println(mt2.pose);
            // swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,  VecBuilder.fill(0.7,0.7,9999999));
            // System.out.println(mt2.pose);
            // add here putting vision measurement on dashboard
            // System.out.println("good");
        }
    }

    // returns true if red alliance, false if blue alliance and if no alliance
    public boolean isRedAlliance(){
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    // posts the current trajectory the robot is following on dashboard
    public void postTrajectory(Trajectory trajectory)
    {
      swerveDrive.postTrajectory(trajectory);
    }

    // sets the motors to coast/brake
    public void setMotorsBrake(boolean isBrake){
        swerveDrive.setMotorIdleMode(isBrake);
    }

    public ChassisSpeeds getFieldVelocity(){
        return swerveDrive.getFieldVelocity();
    }
    
    public SwerveDrive getSwerveDriveObject(){
        return swerveDrive;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public ChassisSpeeds getFieldRelativeSpeeds() {
        
        return swerveDrive.getFieldVelocity();
    }


}

