package frc.robot.subsystems.indication;

import com.ctre.phoenix6.controls.SolidColor;
import org.littletonrobotics.junction.AutoLog;

public interface IndicatorsIO {
  /** Logged indicator inputs/outputs needed for replay parity. */
  @AutoLog
  public static class IndicatorsIOInputs {
    public boolean candleConnected = false;
    public String activePattern = "none";
  }

  /** Updates current indicator input/output state. */
  public default void updateInputs(IndicatorsIOInputs inputs) {}

  /** Sets CANdle output request. */
  public default void setControl(SolidColor color) {}
}
