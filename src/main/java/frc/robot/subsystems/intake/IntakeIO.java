package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  /** Logged motor/sensor data for the intake rollers and arm. */
  @AutoLog
  public static class IntakeIOInputs {
    public boolean rollersConnected = false;
    public double rollersPositionRad = 0.0;
    public double rollersVelocityRadPerSec = 0.0;
    public double rollersAppliedVolts = 0.0;
    public double rollersCurrentAmps = 0.0;
    public double rollersTempCelsius = 0.0;

    public boolean armConnected = false;
    public double armPositionRad = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double armAppliedVolts = 0.0;
    public double armCurrentAmps = 0.0;
    public double armTempCelsius = 0.0;
  }

  /** Updates the complete set of loggable intake inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Open-loop rollers output [-1,1]. */
  public default void setRollersOpenLoop(double output) {}

  /** Open-loop arm voltage output in volts. */
  public default void setArmVoltage(double volts) {}

  /** Closed-loop arm position target in degrees. */
  public default void setArmPositionDegrees(double positionDegrees) {}

  /** Configures arm position-loop proportional gain. */
  public default void configureArmPositionKp(double kP) {}
}
