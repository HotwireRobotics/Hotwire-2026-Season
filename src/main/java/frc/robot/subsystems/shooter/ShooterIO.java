package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    // Feeder
    public boolean feederConnected = false;
    public double feederVelocityRpm = 0.0;
    public double feederVoltage = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederTempCelsius = 0.0;

    // Left shooter (lead)
    public boolean leftShooterConnected = false;
    public double leftShooterVelocityRpm = 0.0;
    public double leftShooterVoltage = 0.0;
    public double leftShooterCurrentAmps = 0.0;
    public double leftShooterTempCelsius = 0.0;

    // Right shooter (lead)
    public boolean rightShooterConnected = false;
    public double rightShooterVelocityRpm = 0.0;
    public double rightShooterVoltage = 0.0;
    public double rightShooterCurrentAmps = 0.0;
    public double rightShooterTempCelsius = 0.0;

    // Left follower
    public double leftFollowerVelocityRpm = 0.0;
    public double leftFollowerVoltage = 0.0;
    public double leftFollowerCurrentAmps = 0.0;
    public double leftFollowerTempCelsius = 0.0;

    // Right follower
    public double rightFollowerVelocityRpm = 0.0;
    public double rightFollowerVoltage = 0.0;
    public double rightFollowerCurrentAmps = 0.0;
    public double rightFollowerTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. On replay, Logger fills inputs from the log. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run the feeder at the specified output (-1 to 1). */
  public default void setFeederOutput(double output) {}

  /** Run the feeder at the specified velocity. */
  public default void setFeederVelocity(AngularVelocity velocity) {}

  /** Run shooter(s) at the specified output (-1 to 1). Device is LEFT, RIGHT, or BOTH. */
  public default void setShooterOutput(Device device, double output) {}

  /** Run shooter(s) at the specified velocity. Device is LEFT, RIGHT, or BOTH. */
  public default void setShooterVelocity(Device device, AngularVelocity velocity) {}

  /** Run shooter(s) at the specified voltage (e.g. for SysId). Device is LEFT or RIGHT. */
  public default void setShooterVoltage(Device device, Voltage voltage) {}

  /** Which shooter device(s) to command. */
  public enum Device {
    FEEDER,
    RIGHT,
    LEFT,
    BOTH
  }
}
