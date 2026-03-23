package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

public class Logs {
  /**
   * Log TalonFX data to the Logger. This should be called periodically in the subsystem's periodic
   * method.
   *
   * @param motor
   */
  public static void log(Object fix, Motor motor) {
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Position",
        motor.getPosition().getValue().in(Rotations),
        "rotations");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Velocity",
        motor.getVelocity().getValue().in(RotationsPerSecond),
        "RPS");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Acceleration",
        motor.getAcceleration().getValue().in(RotationsPerSecondPerSecond),
        "RPS/s");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/SupplyVoltage",
        motor.getSupplyVoltage().getValue().in(Volts),
        "V");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/SupplyCurrent",
        motor.getSupplyCurrent().getValue().in(Amps),
        "A");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/StatorCurrent",
        motor.getStatorCurrent().getValue().in(Amps),
        "A");
    Logger.recordOutput(
        "Motors/" + motor.getSubsystem().getName() + '/' + fix + "/Temperature",
        motor.getDeviceTemp().getValue().in(Celsius),
        "C");
  }

  public static void log(Motor motor) {
    log(motor.getDeviceID(), motor);
  }

  /**
   * Log subsystem state to the Logger. This should be called periodically in the subsystem's
   * periodic method.
   *
   * @param subsystem
   * @param state
   */
  public static void log(Subsystem subsystem, Object state) {
    Logger.recordOutput("States/" + subsystem.getName() + '/', state.toString());
  }

  /**
   * Log any key-value pair to the Logger. This can be used for logging miscellaneous data that
   * doesn't fit into the other categories. This should be called periodically in the subsystem's
   * periodic method.
   *
   * @param key
   * @param value
   */
  public static void write(String key, Object value) {
    Logger.recordOutput(key, value.toString());
  }
}
