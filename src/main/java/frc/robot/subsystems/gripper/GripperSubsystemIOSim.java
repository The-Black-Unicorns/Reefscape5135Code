package frc.robot.subsystems.gripper;

import static edu.wpi.first.units.Units.Centimeter;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.units.measure.Distance;
import frc.robot.SuperStructure;

public class GripperSubsystemIOSim implements GripperSubsystemIO {
    private final IntakeSimulation intakeSimulation;
    private final SuperStructure superStructure;
    public GripperSubsystemIOSim(SuperStructure structure) {
        this.superStructure = structure;
        this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
            "Coral",
            structure.swerve.getSimDrive(),
            Distance.ofRelativeUnits(37, Centimeter),
            Distance.ofRelativeUnits(15, Centimeter),
            IntakeSide.FRONT,
            1
        );
    }

    @Override
    public void setGripperMotorSpeed(double speed) {
        if (speed == 0) {
            System.out.println("Stopping intake");
            intakeSimulation.stopIntake();
        } else if (speed > 0 && !isNoteInsideIntake()) {
            System.out.println("Starting intake");
            intakeSimulation.startIntake();
        } else if (speed < 0 && isNoteInsideIntake()) {
            System.out.println("Ejecting coral");
            intakeSimulation.obtainGamePieceFromIntake();
            intakeSimulation.stopIntake();
            superStructure.swerve.scoreL1Simulation();
            
        }
    }

    public boolean isNoteInsideIntake() {
        return intakeSimulation.getGamePiecesAmount() != 0; // True if there is a game piece in the intake
    }



}
