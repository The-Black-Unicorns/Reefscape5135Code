package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import org.dyn4j.geometry.Rotation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.MomentOfInertiaUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystemSim extends SubsystemBase implements SwerveSubsystemIO {

    private final SelfControlledSwerveDriveSimulation simulatedDrive;

StructArrayPublisher<Pose3d> coralPoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("GamePieces/CoralPiecesArray", Pose3d.struct)
      .publish();
      StructArrayPublisher<Pose3d> algaePoses = NetworkTableInstance.getDefault()
      .getStructArrayTopic("GamePieces/AlgaePiecesArray", Pose3d.struct)
      .publish();


    public SwerveSubsystemSim() {

     

        @SuppressWarnings("unchecked")
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
            .withBumperSize(Distance.ofRelativeUnits(70, Centimeters), Distance.ofRelativeUnits(70, Centimeters))
            .withGyro(() -> new GyroSimulation(0.01, 0.02))
            .withRobotMass(Mass.ofRelativeUnits(90, Pounds))
            .withSwerveModules(() -> new SwerveModuleSimulation( new SwerveModuleSimulationConfig(
                DCMotor.getFalcon500Foc(1),
                DCMotor.getFalcon500(1), //drive motor model
                 //drive gear ratio
                5.9, //steer gear ratio
                18.75, //drive min friction voltage
                Voltage.ofRelativeUnits(0.1, Volts), //steer min friction voltage
                Voltage.ofRelativeUnits(0.1, Volts), //wheel radius
                Distance.ofRelativeUnits(1.5, Inch), // steer rotational inertia
                MomentOfInertia.ofRelativeUnits(0.01, KilogramSquareMeters), // wheel friction coefficient
                1.5
            )));

            simulatedDrive = new SelfControlledSwerveDriveSimulation(new SwerveDriveSimulation(config, new Pose2d(2, 2, new Rotation2d())));


            SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());
            SimulatedArena.getInstance().resetFieldForAuto();
            

    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        simulatedDrive.runChassisSpeeds(
            new ChassisSpeeds(translation.getX(), translation.getY(), rotation),
            new Translation2d(),
            fieldRelative,
            true);
            DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue).equals(DriverStation.Alliance.Red);
    }

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        simulatedDrive.runSwerveStates(states);
    }

    @Override
    public SwerveModulePosition[] getModulePositions(){
        return simulatedDrive.getLatestModulePositions();
    }

    @Override
    public SwerveModuleState[] getModuleStates() {
        return simulatedDrive.getMeasuredStates();
    }

    @Override
    public Rotation2d getGyroYaw() {
        return simulatedDrive.getOdometryEstimatedPose().getRotation();
    }

    @Override
    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    @Override
    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }

    @Override
    public void periodic() {
        simulatedDrive.periodic();
        Pose3d[] coralPosesArray = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Coral");
        Pose3d[] algaePosesArray = SimulatedArena.getInstance()
            .getGamePiecesArrayByType("Algae");
        coralPoses.accept(coralPosesArray);
        algaePoses.accept(algaePosesArray);
    }

    @Override
    public AbstractDriveTrainSimulation getSimDrive() {
        return simulatedDrive.getDriveTrainSimulation();
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return simulatedDrive.getActualSpeedsRobotRelative();
    }

    @Override
    public void simulationPeriodic(){
        SimulatedArena.getInstance().simulationPeriodic();
    }

    @Override
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return simulatedDrive.getActualSpeedsFieldRelative();
    }

    @Override
    public void scoreL1Simulation() {
        SimulatedArena.getInstance()
               .addGamePieceProjectile(new ReefscapeCoralOnFly(
        // Obtain robot position from drive simulation
        getPose().getTranslation(),
        // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
        new Translation2d(0.35, 0),
        // Obtain robot speed from drive simulation
        getFieldRelativeSpeeds(),
        // Obtain robot facing from drive simulation
        getPose().getRotation(),
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
                Distance.ofRelativeUnits(1.5, Meter),
                // The initial speed ofzz the coral
                LinearVelocity.ofRelativeUnits(1.5, MetersPerSecond),
                // The coral is ejected at a 35-degree slope
                Angle.ofRelativeUnits(-45, Degree)
            )
        );
    }
}
