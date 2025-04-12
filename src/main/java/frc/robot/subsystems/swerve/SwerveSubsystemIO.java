package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveSubsystemIO {

    // public static class SwerveIOInputs {
    //     ChassisSpeeds speeds = new ChassisSpeeds();
    //     Rotation2d gyroYaw = new Rotation2d();
    //     Pose2d pose = new Pose2d();

    // }

    // default void updateInputs(SwerveIOInputs inputs) {}

    default void drive(Translation2d speeds, double rotation, boolean fieldRelative, boolean isOpenLoop) {}

    default void setModuleStates(SwerveModuleState[] states) {}

    default SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[0];
    }

    default SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[0];
    }

    default void setPose(Pose2d pose) {}

    default Rotation2d getHeading() {return new Rotation2d();}

    default void setHeading(Rotation2d heading) {}

    default void zeroHeading() {}

    default void zeroGyroWithAlliance() {}

    Rotation2d getGyroYaw();

    Pose2d getPose();

    ChassisSpeeds getRobotRelativeSpeeds();

    ChassisSpeeds getFieldRelativeSpeeds();

    default void updateOdometry() {}

    default void periodic() {}

    public default void updateVisionReading(DoubleSupplier robotYaw, DoubleSupplier robotYawRate) {updateVisionReading();}
    
    public default void updateVisionReading() {}

    public default Pose2d getActualPoseSim() {return getPose();}

    public default AbstractDriveTrainSimulation getSimDrive() {return null;}

    public default void scoreL1Simulation() {}

    public default void hpDropCoralSimulation() {}

    public default void simulationPeriodic() {}



    


}
