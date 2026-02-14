package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO layer for the hopper subsystem. Isolates hardware so inputs can be logged and replayed per
 * AdvantageKit IO Interfaces:
 * https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces
 */
public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean upperConnected = false;
    public double upperPositionRot = 0.0;
    public double upperVelocityRpm = 0.0;
    public double upperAppliedVolts = 0.0;
    public double upperCurrentAmps = 0.0;
    public double upperTempCelsius = 0.0;

    public boolean lowerConnected = false;
    public double lowerPositionRot = 0.0;
    public double lowerVelocityRpm = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double lowerCurrentAmps = 0.0;
    public double lowerTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Run upper feed motor at the specified output (-1 to 1). */
  public default void setUpperOutput(double output) {}

  /** Run lower feed motor at the specified output (-1 to 1). */
  public default void setLowerOutput(double output) {}
}
