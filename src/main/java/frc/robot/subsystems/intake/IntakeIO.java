package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean rollersConnected = false;
    public double rollersVelocityRpm = 0.0;
    public double rollersVoltage = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double rollersTempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. On replay, Logger fills inputs from the log. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run the rollers at the specified output (-1 to 1). */
  public default void setRollersOutput(double output) {}
}
