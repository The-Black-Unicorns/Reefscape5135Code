package frc.robot.subsystems.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.ControllerConstants.STICK_DEADBAND;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveSubsystemIO io;
    // protected final SwerveSubsystemIO.SwerveIOInputs inputs;

    public SwerveSubsystem(SwerveSubsystemIO io) {
        this.io = io;
        // this.inputs = new SwerveSubsystemIO.SwerveIOInputs();
    }

    public void drive(Translation2d speeds, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        io.drive(speeds, rotation, fieldRelative, isOpenLoop);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        io.setModuleStates(states);
    }

    public SwerveModulePosition[] getModulePositions() {
        return io.getModulePositions();
    }

    public SwerveModuleState[] getModuleStates() {
        return io.getModuleStates();
    }

    public void setPose(Pose2d pose) {
        io.setPose(pose);
    }

    public Pose2d getPose() {
        return io.getPose();
    }

    public Rotation2d getHeading() {
        return io.getHeading();
    }

    public void setHeading(Rotation2d heading) {
        io.setHeading(heading);
    }

    public void zeroHeading() {
        io.zeroHeading();
    }

    public Rotation2d getGyroYaw() {
        return io.getGyroYaw();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return io.getRobotRelativeSpeeds();
    }

    public void zeroGyroWithAlliance() {
        io.zeroGyroWithAlliance();
    }

    public Command driveCommandForDriver(DoubleSupplier xSpeed, DoubleSupplier ySpeed,
            DoubleSupplier angularSpeed, BooleanSupplier isFieldOriented, DoubleSupplier speedExponent) {
        return new RunCommand(() ->

        io.drive(
            new Translation2d(
                MathUtil.applyDeadband(xSpeed.getAsDouble(), STICK_DEADBAND),
                MathUtil.applyDeadband(ySpeed.getAsDouble(), STICK_DEADBAND)).times(MAX_SPEED).times(speedExponent.getAsDouble() * 
                Math.abs(speedExponent.getAsDouble())),
            MathUtil.applyDeadband(angularSpeed.getAsDouble(), STICK_DEADBAND)  * MAX_ANGULAR_VELOCITY
            ,
            isFieldOriented.getAsBoolean(),
            false),//check if closed loop is better then open loop

        this);
    }

    public Command driveToPose(Pose2d pose){
        PathConstraints constraints = new PathConstraints(
            4, 7, 5, 6);

            return AutoBuilder.pathfindToPose(pose, constraints, 0);
    }

    public void updateOdometry() {
        io.updateOdometry();
    }

    private StructPublisher<Pose2d> curPosePublisher = 
    NetworkTableInstance.getDefault()
    .getStructTopic("Current Pose", Pose2d.struct).publish();

    private StructPublisher<Pose2d> curActualPosePublisher = 
    NetworkTableInstance.getDefault()
    .getStructTopic("Current Actual Pose (Sim)", Pose2d.struct).publish();

    @Override
    public void periodic() {
        io.periodic();

        curPosePublisher.set(getPose());
        curActualPosePublisher.set(getActualPoseSim());
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public AbstractDriveTrainSimulation getSimDrive() {
        return io.getSimDrive();
    }

    public void scoreL1Simulation() {
        io.scoreL1Simulation();
    }

    public void hpDropCoralSimulation() {
        io.hpDropCoralSimulation();
    }

    public Pose2d getActualPoseSim() {
        return io.getActualPoseSim();
    }

    public void updateLimelightReading(DoubleSupplier robotYaw, DoubleSupplier robotYawRate) {
        io.updateVisionReading(robotYaw, robotYawRate);
    }

    public void updateLimelightReading() {
        io.updateVisionReading();
    }

    public ChassisSpeeds getFieldRelativeSpeeds() {
        return io.getFieldRelativeSpeeds();
    }

    public void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        io.setModuleStates(states);
    }

    public SwerveSubsystemIO getIO() {
        return io;
    }

    public void setIO(SwerveSubsystemIO io) {
        this.io = io;
    }

}
