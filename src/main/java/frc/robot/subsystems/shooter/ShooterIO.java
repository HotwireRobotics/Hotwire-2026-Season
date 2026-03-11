package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  /** Logged IO inputs for all shooter motors. */
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftShooterConnected = false;
    public double leftShooterPositionRad = 0.0;
    public double leftShooterVelocityRadPerSec = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double leftShooterCurrentAmps = 0.0;
    public double leftShooterTempCelsius = 0.0;

    public boolean rightShooterConnected = false;
    public double rightShooterPositionRad = 0.0;
    public double rightShooterVelocityRadPerSec = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double rightShooterCurrentAmps = 0.0;
    public double rightShooterTempCelsius = 0.0;

    public boolean feederConnected = false;
    public double feederPositionRad = 0.0;
    public double feederVelocityRadPerSec = 0.0;
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
    public double feederTempCelsius = 0.0;
  }

  /** Updates the complete set of loggable shooter inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Open-loop output [-1,1]. */
  public default void setLeftShooterOpenLoop(double output) {}

  /** Open-loop output [-1,1]. */
  public default void setRightShooterOpenLoop(double output) {}

  /** Open-loop output [-1,1]. */
  public default void setFeederOpenLoop(double output) {}

  /** Closed-loop velocity in rad/s. */
  public default void setLeftShooterVelocity(double velocityRadPerSec) {}

  /** Closed-loop velocity in rad/s. */
  public default void setRightShooterVelocity(double velocityRadPerSec) {}

  /** Closed-loop velocity in rad/s. */
  public default void setFeederVelocity(double velocityRadPerSec) {}

  /** Voltage control in volts. */
  public default void setLeftShooterVoltage(double volts) {}

  /** Voltage control in volts. */
  public default void setRightShooterVoltage(double volts) {}

  /** Voltage control in volts. */
  public default void setFeederVoltage(double volts) {}

  /** Configure proportional gain for velocity slot. */
  public default void configureSlot0Kp(double kP) {}

  /** Helper conversion from RPS to rad/s for callers that already use RPS. */
  public static double rpsToRadPerSec(double rps) {
    return Units.rotationsToRadians(rps);
  }
}
