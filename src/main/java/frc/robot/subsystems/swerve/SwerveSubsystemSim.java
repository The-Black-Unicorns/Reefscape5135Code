package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import java.io.File;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Swerve;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystemSim /*extends SubsystemBase*/ implements SwerveSubsystemIO {

    // private final SwerveDriveSimulation simulatedDrive;
    private final SwerveDrive swerveDrive;
    private boolean doRejectUpdate;
    private PhotonCamera camera;
    private PhotonCameraSim cameraSim;
    private SimCameraProperties cameraProps;
    private VisionSystemSim visionSim;
    private final AprilTagFieldLayout aprilTagFieldLayout;

StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SmartDashboard/MapleArenaSimulation/GamePieces/CoralPiecesArray", Pose3d.struct)
      .publish();
      StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SmartDashboard/MapleArenaSimulation/GamePieces/AlgaePiecesArray", Pose3d.struct)
      .publish();


    public SwerveSubsystemSim(File directory) {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        try {
            swerveDrive = new SwerveParser(directory)
                .createSwerveDrive(Swerve.MAX_SPEED,
                    new Pose2d(2, 2, new Rotation2d()));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
        swerveDrive.pushOffsetsToEncoders();


        // @SuppressWarnings("unchecked")
        // final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
        //     .withBumperSize(Distance.ofRelativeUnits(70, Centimeters), Distance.ofRelativeUnits(70, Centimeters))
        //     .withGyro(() -> new GyroSimulation(0.01, 0.02))
        //     .withRobotMass(Mass.ofRelativeUnits(90, Pounds))
        //     .withSwerveModules(() -> new SwerveModuleSimulation( new SwerveModuleSimulationConfig(
        //         DCMotor.getFalcon500Foc(1),
        //         DCMotor.getFalcon500(1), //drive motor model
        //          //drive gear ratio
        //         5.9, //steer gear ratio
        //         18.75, //drive min friction voltage
        //         Voltage.ofRelativeUnits(0.1, Volts), //steer min friction voltage
        //         Voltage.ofRelativeUnits(0.1, Volts), //wheel radius
        //         Distance.ofRelativeUnits(1.5, Inch), // steer rotational inertia
        //         MomentOfInertia.ofRelativeUnits(0.05, KilogramSquareMeters), // wheel friction coefficient
        //         1.5
        //     )));

            // simulatedDrive = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(config, new Pose2d(2, 2, new Rotation2d())));
            doRejectUpdate = false;
            SimulatedArena.getInstance().resetFieldForAuto();

            aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
            if (Robot.isSimulation()){
                visionSim = new VisionSystemSim("main");
                visionSim.addAprilTags(aprilTagFieldLayout);

                cameraProps = new SimCameraProperties();
                cameraProps.setCalibError(0.25, 0.08);
                cameraProps.setAvgLatencyMs(35);
                cameraProps.setLatencyStdDevMs(5);
                cameraProps.setFPS(30);

                camera = new PhotonCamera("Cam1");
                cameraSim = new PhotonCameraSim(camera, cameraProps);

                visionSim.addCamera(cameraSim, new Transform3d(new Translation3d(0.3, 0, 0.1), new Rotation3d(0,25,0)));


            }
    }


    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        // simulatedDrive.runChassisSpeeds(
        //     new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
        //     new Translation2d(),
        //     fieldRelative,
        //     true);
        //     DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
        
        swerveDrive.drive(translation,rotation,fieldRelative,isOpenLoop);
    }

    

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        // simulatedDrive.runSwerveStates(states);
        swerveDrive.setModuleStates(states, true);
    }

    @Override
    public SwerveModulePosition[] getModulePositions(){
        // return simulatedDrive.getLatestModulePositions();
        return swerveDrive.getModulePositions();
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        // return simulatedDrive.getMeasuredStates();
        return swerveDrive.getStates();
    }

    @Override
    public Rotation2d getGyroYaw() {
        // return simulatedDrive.getOdometryEstimatedPose().getRotation();
        return swerveDrive.getYaw();
    }

    @Override
    public Pose2d getPose() {
        // return simulatedDrive.getOdometryEstimatedPose();
        return swerveDrive.getPose();
    }

    @Override
    public Pose2d getActualPoseSim() {
        // return simulatedDrive.getActualPoseInSimulationWorld();
        if (swerveDrive.getSimulationDriveTrainPose().isPresent()) {
            return swerveDrive.getSimulationDriveTrainPose().get();
        }
        else {
            System.out.println("No simulation drive train pose");
            return getPose();
            
        }
        
    }

    @Override
    public void setPose(Pose2d pose) {
        // simulatedDrive.setSimulationWorldPose(pose);
        // simulatedDrive.resetOdometry(pose);
        swerveDrive.resetOdometry(pose);
        
    }

    @Override
    public void periodic() {
        updateOdometry();
        visionSim.update(getActualPoseSim());
        updateVisionReading();
        // simulatedDrive.periodic();

        Pose3d[] coralPosesArray = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral");
        Pose3d[] algaePosesArray = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Algae");
        coralPoses.accept(coralPosesArray);
        algaePoses.accept(algaePosesArray);
    }

    @Override
    public AbstractDriveTrainSimulation getSimDrive() {
        return swerveDrive.getMapleSimDrive().orElse(null);
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
        // return simulatedDrive.getRobotRelativeSpeeds();
    }

    @Override
    public void simulationPeriodic(){
        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public ChassisSpeeds getFieldRelativeSpeeds() {
        
        // return simulatedDrive.getActualSpeedsFieldRelative();
        return swerveDrive.getFieldVelocity();
    }

    @Override
    public void scoreL1Simulation() {
        SimulatedArena.getInstance()
               .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        getActualPoseSim().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.35, 0),
        // Obtain robot speed from drive simulation
        getFieldRelativeSpeeds(),
        // Obtain robot facing from drive simulation
        getActualPoseSim().getRotation(),
        // The height at which the coral is ejected
        Distance.ofRelativeUnits(0.75, Meter),
        // The initial speed ofzz the coral
        LinearVelocity.ofRelativeUnits(1, MetersPerSecond),
        // The coral is ejected at a 35-degree slope
        Angle.ofRelativeUnits(0, Degree)
        ));
    }

    @Override
    public void hpDropCoralSimulation() {
        SimulatedArena.getInstance()
            .addGamePieceProjectile(new ReefscapeCoralOnFly(
                // Obtain robot position from drive simulation
                new Translation2d(1, 1),
                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                new Translation2d(0, 0),
                // Obtain robot speed from drive simulation
                new ChassisSpeeds(),
                // Obtain robot facing from drive simulation
                new Rotation2d(58),
                // The height at which the coral is ejected
                Distance.ofRelativeUnits(2, Meter),
                // The initial speed ofzz the coral
                LinearVelocity.ofRelativeUnits(1.5, MetersPerSecond),
                // The coral is ejected at a 35-degree slope
                Angle.ofRelativeUnits(-45, Degree)
            )
        );
    }

    @Override
    public void zeroGyroWithAlliance()
    {
    if(!DriverStation.getAlliance().orElse(null).equals(DriverStation.Alliance.Red)){
        zeroHeading();
        setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else{
        zeroHeading();
        setPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    }
    }

    @Override
    public void zeroHeading() {
        swerveDrive.zeroGyro();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        // simulatedDrive.setGyroAngle(heading);
        swerveDrive.resetOdometry(new Pose2d(getPose().getTranslation(), heading));
    }

    @Override
    public Rotation2d getHeading() {
        // return simulatedDrive.getGyroAngle();
        return swerveDrive.getYaw();
    }

    @Override
    public void updateOdometry() {
        // simulatedDrive.updateOdometry();
        swerveDrive.updateOdometry();
    }

    @Override
    public void updateVisionReading() {
        // simulatedDrive.updateLimelightReading(robotYaw, robotYawRate);
        swerveDrive.addVisionMeasurement(visionSim.getRobotPose().toPose2d(), Timer.getFPGATimestamp());
    }



}
