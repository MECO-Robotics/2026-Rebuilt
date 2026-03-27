/**
 * Position-joint subsystem package.
 *
 * <p>
 * Contains subsystem behavior and IO implementations for position-controlled
 * mechanisms such as the intake rack and shooter hood.
 *
 * <p>
 * Major types include:
 *
 * <ul>
 * <li>{@link frc.robot.subsystems.position_joint.PositionJoint}:
 * subsystem-level behavior, profiling, and tunable gains.
 * <li>{@link frc.robot.subsystems.position_joint.PositionJointIO}: hardware
 * abstraction interface.
 * <li>Hardware-specific IO implementations for TalonFX, Spark Max, and
 * simulation backends.
 * <li>{@link frc.robot.constants.types.PositionJointConstants}: shared gains
 * and hardware configs.
 * </ul>
 */
package frc.robot.subsystems.position_joint;
