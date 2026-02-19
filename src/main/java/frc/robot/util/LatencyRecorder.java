package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

/**
 * Records timestamps along the control pipeline so we can measure latency from driver input to
 * mechanism actuation, motion start, and action completion. All times use FPGA timestamp for
 * consistency. Logged under "Latency/Control/..." for analysis in AdvantageScope.
 */
public final class LatencyRecorder {

  private static double triggerTimeSec = 0;
  private static String triggerName = "";

  private LatencyRecorder() {}

  /**
   * Call when the driver input is recognized (e.g. command scheduled / button rising edge). Resets
   * the current trigger and sets the reference time for subsequent delta calculations.
   *
   * @param name Identifier for this action (e.g. "Shooter", "Intake")
   */
  public static void recordTrigger(String name) {
    triggerTimeSec = Timer.getFPGATimestamp();
    triggerName = name;
    Logger.recordOutput("Latency/Control/" + name + "/TriggerTimeSec", triggerTimeSec);
  }

  /**
   * Call when the mechanism receives a non-zero command (motor output / setControl). Logs the
   * delay from trigger in ms. Only logs if the last trigger name matches.
   *
   * @param name Must match the name passed to recordTrigger
   * @return Delay from trigger to actuation in ms, or -1 if name mismatch / no trigger
   */
  public static double recordActuation(String name) {
    if (!name.equals(triggerName)) {
      return -1;
    }
    double now = Timer.getFPGATimestamp();
    double ms = (now - triggerTimeSec) * 1000.0;
    Logger.recordOutput("Latency/Control/" + name + "/InputToActuationMs", ms);
    return ms;
  }

  /**
   * Call when the mechanism physically starts moving (e.g. velocity first exceeds threshold). Logs
   * the delay from trigger in ms.
   *
   * @param name Must match the name passed to recordTrigger
   * @return Delay from trigger to motion start in ms, or -1 if name mismatch
   */
  public static double recordMotionStart(String name) {
    if (!name.equals(triggerName)) {
      return -1;
    }
    double now = Timer.getFPGATimestamp();
    double ms = (now - triggerTimeSec) * 1000.0;
    Logger.recordOutput("Latency/Control/" + name + "/InputToMotionMs", ms);
    return ms;
  }

  /**
   * Call when the action is complete (e.g. shot fired, note in position). Logs the delay from
   * trigger in ms.
   *
   * @param name Must match the name passed to recordTrigger
   * @return Delay from trigger to completion in ms, or -1 if name mismatch
   */
  public static double recordActionComplete(String name) {
    if (!name.equals(triggerName)) {
      return -1;
    }
    double now = Timer.getFPGATimestamp();
    double ms = (now - triggerTimeSec) * 1000.0;
    Logger.recordOutput("Latency/Control/" + name + "/InputToCompleteMs", ms);
    return ms;
  }
}
