package frc.robot.subsystems.indication;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;

/** Real CANdle IO implementation for LED indicators. */
public class IndicatorsIOCANdle implements IndicatorsIO {
  private final CANdle candle = new CANdle(0);
  private String activePattern = "none";

  @Override
  public void updateInputs(IndicatorsIOInputs inputs) {
    inputs.candleConnected = true;
    inputs.activePattern = activePattern;
  }

  @Override
  public void setControl(SolidColor color) {
    candle.setControl(color);
    activePattern = "solid";
  }
}
