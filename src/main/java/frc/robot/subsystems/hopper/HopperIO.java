package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

/** Defines the hardware abstraction for hopper control and telemetry. */
public interface HopperIO {
  /** Loggable hopper inputs captured during each periodic loop. */
  @AutoLog
  public static class HopperIOInputs {
    public boolean connected = false;
    public double positionRot = 0.0;
    public double velocityRpm = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempC = 0.0;
  }

  /** Updates the latest hardware/sim telemetry fields. */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Sets hopper motor percent output in the range -1..1. */
  public default void setHopperOpenLoop(double output) {}
}
