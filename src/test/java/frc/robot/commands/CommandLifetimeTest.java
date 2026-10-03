package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.ShooterCommands;
import frc.robot.constants.subsystems.IntakeConstants;
import frc.robot.constants.subsystems.ShooterConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointIO;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class CommandLifetimeTest {
	private static final AtomicInteger NEXT_ID = new AtomicInteger();
	private final CommandScheduler scheduler = CommandScheduler.getInstance();

	@BeforeAll
	static void initializeHal() {
		HAL.initialize(500, 0);
	}

	@BeforeEach
	void enableRobot() {
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
	}

	@AfterEach
	void cleanScheduler() {
		scheduler.cancelAll();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		DriverStation.refreshData();
	}

	@Test
	void flywheelHoldCommandDoesNotFinishAtSetpoint() {
		FakeFlywheelIO io = new FakeFlywheelIO();
		Flywheel flywheel = new Flywheel(io, ShooterConstants.FLYWHEEL_ROLLER_GAINS);
		Command hold = Flywheel.holdVelocity(flywheel, () -> 30.0);

		scheduler.schedule(hold);
		scheduler.run();
		io.velocity = 30.0;
		scheduler.run();

		assertTrue(hold.isScheduled());
		assertTrue(io.velocityControlActive);
		assertEquals(30.0, io.desiredVelocity, 1e-9);

		scheduler.cancel(hold);
		assertFalse(io.velocityControlActive);
		assertEquals(0.0, io.voltage, 1e-9);
		scheduler.unregisterSubsystem(flywheel);
	}

	@Test
	void shooterPresetStopsWithZeroVoltageWhenReleased() {
		FakeFlywheelIO shooterIo = new FakeFlywheelIO();
		FakePositionJointIO hoodIo = new FakePositionJointIO();
		Flywheel shooter = new Flywheel(shooterIo, ShooterConstants.FLYWHEEL_ROLLER_GAINS);
		PositionJoint hood = new PositionJoint(hoodIo, ShooterConstants.HOOD_GAINS);
		Command preset = ShooterCommands.hubPreset(shooter, hood);

		scheduler.schedule(preset);
		scheduler.run();
		scheduler.run();

		assertTrue(preset.isScheduled());
		assertTrue(shooterIo.velocityControlActive,
				() -> "velocity calls=" + shooterIo.velocityCalls + ", voltage calls=" + shooterIo.voltageCalls);
		assertEquals(ShooterConstants.SHOOTER_PRESET.HUB.get(), shooterIo.desiredVelocity, 1e-9);

		scheduler.cancel(preset);

		assertFalse(shooterIo.velocityControlActive);
		assertEquals(0.0, shooterIo.voltage, 1e-9);
		scheduler.unregisterSubsystem(shooter, hood);
	}

	@Test
	void voltageCommandStopsWithZeroVoltageWhenReleased() {
		FakeFlywheelIO io = new FakeFlywheelIO();
		Flywheel flywheel = new Flywheel(io, IntakeConstants.INTAKE_ROLLER_GAINS);
		Command command = Flywheel.setVoltage(flywheel, () -> 8.0);

		scheduler.schedule(command);
		scheduler.run();
		assertEquals(8.0, io.voltage, 1e-9);

		scheduler.cancel(command);

		assertFalse(io.velocityControlActive);
		assertEquals(0.0, io.voltage, 1e-9);
		scheduler.unregisterSubsystem(flywheel);
	}

	@Test
	void intakeAcquireOwnsEveryMechanismAndStopsPathOnRelease() {
		FakePositionJointIO rackIo = new FakePositionJointIO();
		PositionJoint rack = new PositionJoint(rackIo, IntakeConstants.INTAKE_RACK_GAINS);
		FakeFlywheelIO intakeIo = new FakeFlywheelIO();
		FakeFlywheelIO conveyorIo = new FakeFlywheelIO();
		FakeFlywheelIO bottomIo = new FakeFlywheelIO();
		FakeFlywheelIO topIo = new FakeFlywheelIO();
		Flywheel intake = new Flywheel(intakeIo, IntakeConstants.INTAKE_ROLLER_GAINS);
		Flywheel conveyor = new Flywheel(conveyorIo, ShooterConstants.CONVEYOR_GAINS);
		Flywheel bottom = new Flywheel(bottomIo, ShooterConstants.INDEXER_ROLLER_GAINS);
		Flywheel top = new Flywheel(topIo, ShooterConstants.INDEXER_ROLLER_GAINS);
		Command acquire = IntakeCommands.acquire(rack, intake, conveyor, bottom, top);

		assertEquals(5, acquire.getRequirements().size());
		scheduler.schedule(acquire);
		scheduler.run();
		rackIo.position = IntakeConstants.RACK_PRESETS.DEPLOY.get();
		scheduler.run();
		assertTrue(acquire.isScheduled());

		scheduler.cancel(acquire);
		assertEquals(0.0, intakeIo.voltage, 1e-9);
		assertEquals(0.0, conveyorIo.voltage, 1e-9);
		assertEquals(0.0, bottomIo.voltage, 1e-9);
		assertEquals(0.0, topIo.voltage, 1e-9);
		scheduler.unregisterSubsystem(rack, intake, conveyor, bottom, top);
	}

	@Test
	void positionJointEmergencyStopPersistsUntilAClosedLoopGoalIsRequested() {
		FakePositionJointIO io = new FakePositionJointIO();
		PositionJoint joint = new PositionJoint(io, ShooterConstants.HOOD_GAINS);

		joint.setPosition(0.020);
		joint.periodic();
		assertTrue(io.positionControlActive);

		joint.stop();
		joint.periodic();
		joint.periodic();
		assertFalse(io.positionControlActive);
		assertEquals(0.0, io.voltage, 1e-9);
		assertTrue(joint.isOpenLoopMode());

		joint.setPosition(0.010);
		joint.periodic();
		assertTrue(io.positionControlActive);
		assertFalse(joint.isOpenLoopMode());
		scheduler.unregisterSubsystem(joint);
	}

	private static final class FakeFlywheelIO implements FlywheelIO {
		private final String name = "TestFlywheel" + NEXT_ID.incrementAndGet();
		double velocity;
		double desiredVelocity;
		double voltage;
		boolean velocityControlActive;
		int velocityCalls;
		int voltageCalls;

		@Override
		public void updateInputs(FlywheelIOInputs inputs) {
			inputs.velocity = velocity;
			inputs.desiredVelocity = desiredVelocity;
		}

		@Override
		public void setVelocity(double velocity) {
			velocityCalls++;
			desiredVelocity = velocity;
			velocityControlActive = true;
		}

		@Override
		public void setVoltage(double voltage) {
			voltageCalls++;
			this.voltage = voltage;
			desiredVelocity = 0.0;
			velocityControlActive = false;
		}

		@Override
		public String getName() {
			return name;
		}
	}

	private static final class FakePositionJointIO implements PositionJointIO {
		private final String name = "TestJoint" + NEXT_ID.incrementAndGet();
		double position;
		double voltage;
		boolean positionControlActive;

		@Override
		public void updateInputs(PositionJointIOInputs inputs) {
			inputs.outputPosition = position;
		}

		@Override
		public void setPosition(double position, double velocity) {
			positionControlActive = true;
		}

		@Override
		public void setVoltage(double voltage) {
			this.voltage = voltage;
			positionControlActive = false;
		}

		@Override
		public String getName() {
			return name;
		}
	}
}
