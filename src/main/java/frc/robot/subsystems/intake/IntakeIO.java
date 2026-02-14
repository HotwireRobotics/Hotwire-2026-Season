package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO layer for the intake subsystem. Isolates hardware so inputs can be logged and replayed per
 * AdvantageKit IO Interfaces:
 * https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean rollersConnected = false;
    public double rollersPositionRot = 0.0;
    public double rollersVelocityRpm = 0.0;
    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double rollersTempCelsius = 0.0;

    public boolean lowerConnected = false;
    public double lowerPositionRot = 0.0;
    public double lowerVelocityRpm = 0.0;
    public double lowerAppliedVolts = 0.0;
    public double lowerCurrentAmps = 0.0;
    public double lowerTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run rollers motor at the specified output (-1 to 1). */
  public default void setRollersOutput(double output) {}

  /** Run lower motor at the specified output (-1 to 1). */
  public default void setLowerOutput(double output) {}
}
