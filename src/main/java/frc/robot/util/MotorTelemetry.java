package frc.robot.util;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;

/** Shared helpers for consistently logging TalonFX telemetry across subsystems. */
public final class MotorTelemetry {
  private static final double SECONDS_PER_MINUTE = 60.0;

  private MotorTelemetry() {}

  /** Logs standard position, velocity, voltage, current, and temperature metrics for a motor. */
  public static void logBasic(String prefix, TalonFX motor) {
    Logger.recordOutput(prefix + "/Position", motor.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        prefix + "/VelocityRPM",
        motor.getVelocity().getValueAsDouble() * SECONDS_PER_MINUTE,
        "rpm");
    Logger.recordOutput(prefix + "/Voltage", motor.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        prefix + "/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(prefix + "/Temperature", motor.getDeviceTemp().getValueAsDouble(), "degC");
  }
}
