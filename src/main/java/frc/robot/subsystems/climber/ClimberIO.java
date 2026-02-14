package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO layer for the climber subsystem. Stub for future hardware; all inputs logged and replayable
 * per AdvantageKit IO Interfaces:
 * https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces
 */
public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean connected = false;
    // Add sensor/motor inputs when hardware is defined (e.g. position, velocity, current).
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}
}
