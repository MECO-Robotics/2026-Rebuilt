package org.mecorobotics.gamepiecevision;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.mecorobotics.gamepiecevision.GamePieceVisionClient.DriveRequest;

/** Command factory that keeps the drivetrain and manual-override policy robot-specific. */
public final class GamePieceDriveCommands {
  private GamePieceDriveCommands() {}

  /**
   * Creates a driver-held pursuit command.
   *
   * <p>The returned command requires the drivetrain subsystem, stops on interruption,
   * and ends when the driver releases the assist control or requests manual override.
   */
  public static Command driverHeldPursuit(
      Subsystem drivetrain,
      GamePieceVisionClient client,
      BooleanSupplier assistHeld,
      BooleanSupplier manualOverride,
      Consumer<DriveRequest> applyRequest,
      Runnable stop) {
    return Commands.runEnd(
            () -> {
              DriveRequest request =
                  client.getRequest(
                      assistHeld.getAsBoolean(), manualOverride.getAsBoolean());
              if (request.active()) {
                applyRequest.accept(request);
              } else {
                stop.run();
              }
            },
            stop,
            drivetrain)
        .onlyWhile(
            () -> assistHeld.getAsBoolean() && !manualOverride.getAsBoolean())
        .withName("DriverHeldGamePiecePursuit");
  }
}
