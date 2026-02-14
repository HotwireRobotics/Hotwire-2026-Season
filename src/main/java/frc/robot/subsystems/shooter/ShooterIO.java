package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO layer for the shooter subsystem. Isolates hardware so inputs can be logged and replayed per
 * AdvantageKit IO Interfaces:
 * https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces Three motors: left
 * shooter, right shooter, feeder (shared).
 */
public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftShooterConnected = false;
    public double leftShooterPositionRot = 0.0;
    public double leftShooterVelocityRpm = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double leftShooterCurrentAmps = 0.0;
    public double leftShooterTempCelsius = 0.0;

    public boolean rightShooterConnected = false;
    public double rightShooterPositionRot = 0.0;
    public double rightShooterVelocityRpm = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double rightShooterCurrentAmps = 0.0;
    public double rightShooterTempCelsius = 0.0;

    public boolean feederConnected = false;
    public double feederPositionRot = 0.0;
    public double feederVelocityRpm = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  // --- Percent output (-1 to 1) ---
  public default void setLeftShooterPercent(double output) {}

  public default void setRightShooterPercent(double output) {}

  public default void setFeederPercent(double output) {}

  // --- Velocity (rot/s) ---
  public default void setLeftShooterVelocityRotPerSec(double rps) {}

  public default void setRightShooterVelocityRotPerSec(double rps) {}

  public default void setFeederVelocityRotPerSec(double rps) {}

  // --- Voltage (for SysId and open-loop) ---
  public default void setLeftShooterVoltage(double volts) {}

  public default void setRightShooterVoltage(double volts) {}

  public default void setFeederVoltage(double volts) {}

  /**
   * Apply Slot0 KP to all shooter/feeder motors (e.g. for dashboard tuning). No-op in sim/replay.
   */
  public default void configureSlot0Kp(double kp) {}
}
