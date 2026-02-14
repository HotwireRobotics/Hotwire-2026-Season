package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hardware IO implementation for hopper (TalonFX upper + lower feed). */
public class HopperIOTalonFX implements HopperIO {
  private final TalonFX upperFeed =
      new TalonFX(Constants.MotorIDs.h_upperFeed, TunerConstants.kCANBus);
  private final TalonFX lowerFeed =
      new TalonFX(Constants.MotorIDs.h_lowerFeed, TunerConstants.kCANBus);

  public HopperIOTalonFX() {}

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.upperConnected = upperFeed.getDeviceTemp().getStatus().isOK();
    inputs.upperPositionRot = upperFeed.getPosition().getValueAsDouble();
    inputs.upperVelocityRpm = upperFeed.getVelocity().getValueAsDouble() * 60.0;
    inputs.upperAppliedVolts = upperFeed.getMotorVoltage().getValueAsDouble();
    inputs.upperCurrentAmps = upperFeed.getSupplyCurrent().getValueAsDouble();
    inputs.upperTempCelsius = upperFeed.getDeviceTemp().getValueAsDouble();

    inputs.lowerConnected = lowerFeed.getDeviceTemp().getStatus().isOK();
    inputs.lowerPositionRot = lowerFeed.getPosition().getValueAsDouble();
    inputs.lowerVelocityRpm = lowerFeed.getVelocity().getValueAsDouble() * 60.0;
    inputs.lowerAppliedVolts = lowerFeed.getMotorVoltage().getValueAsDouble();
    inputs.lowerCurrentAmps = lowerFeed.getSupplyCurrent().getValueAsDouble();
    inputs.lowerTempCelsius = lowerFeed.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setUpperOutput(double output) {
    upperFeed.set(output);
  }

  @Override
  public void setLowerOutput(double output) {
    lowerFeed.set(output);
  }
}
