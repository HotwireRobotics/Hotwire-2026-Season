package frc.robot.subsystems.motors;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorLogs {
  /** Rotor position (rotations). */
  public double position = 0.0;

  /** Rotor velocity (rotations per second). */
  public double velocity = 0.0;

  /** Rotor acceleration (rotations per second squared). */
  public double acceleration = 0.0;

  /** Voltage applied to the motor (V). */
  public double voltage = 0.0;

  /** Supply voltage (V). */
  public double supplyVoltage = 0.0;

  /** Supply current drawn from the bus (A). */
  public double current = 0.0;

  /** Stator current inside the motor (A). */
  public double statorCurrent = 0.0;

  /** Device temperature (°C). */
  public double temperature = 0.0;

  /** Applied output as a normalized value [-1, 1]. */
  public double appliedOutput = 0.0;

  /** Duty cycle output (percentage of full output). */
  public double dutyCycle = 0.0;

  /** True if the motor controller is connected and responding. */
  public boolean connected = true;
}