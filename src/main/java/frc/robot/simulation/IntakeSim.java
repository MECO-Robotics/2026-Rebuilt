package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeSim {

	public static final IntakeSimulation.IntakeSide INTAKE_SIDE = IntakeSimulation.IntakeSide.FRONT;
	public final IntakeSimulation intakeSimulation;

	public IntakeSim(AbstractDriveTrainSimulation driveTrainSimulation) {
		this.intakeSimulation = IntakeSimulation.OverTheBumperIntake(
				// Specify the type of game pieces that the intake can collect
				"Fuel",
				// Specify the drivetrain to which this intake is attached
				driveTrainSimulation,
				// Width of the intake
				Meters.of(0.7),
				// The extension length of the intake beyond the robot's frame (when activated)
				Meters.of(0.2),
				// Remy's CAD intake deploys toward +X.
				INTAKE_SIDE,
				// Simulated hopper capacity
				54);
	}

	public void setRunning(boolean running) {
		if (running) {
			intakeSimulation.startIntake();
		} else {
			intakeSimulation.stopIntake();
		}
	}

	public boolean launchFuel() {
		return intakeSimulation.obtainGamePieceFromIntake();
	}

	public int getStoredFuelCount() {
		return intakeSimulation.getGamePiecesAmount();
	}

	public Command startIntake() {
		return Commands.runOnce(() -> setRunning(true));
	}

	public Command stopIntake() {
		return Commands.runOnce(() -> setRunning(false));
	}
}
