package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Constants;

/** Real hardware implementation of hopper IO using a TalonFX. */
public class HopperIOTalonFX implements HopperIO {
  private final TalonFX hopper;

  /** Creates the hopper IO wrapper for the configured CAN ID. */
  public HopperIOTalonFX() {
    hopper = new TalonFX(Constants.MotorIDs.h_hopper);
  }

  /** Reads live motor telemetry into the hopper inputs struct. */
  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.connected = true;
    inputs.positionRot = hopper.getPosition().getValueAsDouble();
    inputs.velocityRpm = hopper.getVelocity().getValueAsDouble() * 60.0;
    inputs.appliedVolts = hopper.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = hopper.getSupplyCurrent().getValueAsDouble();
    inputs.tempC = hopper.getDeviceTemp().getValueAsDouble();
  }

  /** Commands motor percent output directly on hardware. */
  @Override
  public void setHopperOpenLoop(double output) {
    hopper.set(output);
  }
}
