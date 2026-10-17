package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Rotation2d;

/** A pose-derived stationary shot setpoint. */
public record ShotSolution(double distanceMeters, Rotation2d targetHeading, double hoodRotations,
		double flywheelRotationsPerSecond, boolean calibrated) {
}
