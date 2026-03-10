package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  /** Logged IO inputs for hopper motors/sensors. */
  @AutoLog
  public static class HopperIOInputs {
    public boolean upperConnected = false;
    public double upperPositionRad = 0.0;
    public double upperVelocityRadPerSec = 0.0;
    public double upperAppliedVolts = 0.0;
    public double upperCurrentAmps = 0.0;
    public double upperTempCelsius = 0.0;
  }

  /** Updates all loggable hopper inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Sets upper hopper open-loop output [-1,1]. */
  public default void setUpperOpenLoop(double output) {}
}
