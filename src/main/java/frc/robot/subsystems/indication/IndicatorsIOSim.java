package frc.robot.subsystems.indication;

import com.ctre.phoenix6.controls.SolidColor;

/** Sim/replay-safe indicator IO implementation. */
public class IndicatorsIOSim implements IndicatorsIO {
  private String activePattern = "none";

  @Override
  public void updateInputs(IndicatorsIOInputs inputs) {
    inputs.candleConnected = true;
    inputs.activePattern = activePattern;
  }

  @Override
  public void setControl(SolidColor color) {
    activePattern = "solid";
  }
}
