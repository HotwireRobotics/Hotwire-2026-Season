package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real TalonFX IO implementation for hopper. */
public class HopperIOTalonFX implements HopperIO {
  private final TalonFX upperFeed =
      new TalonFX(Constants.MotorIDs.h_hopperU, TunerConstants.kCANBus);

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    inputs.upperConnected = true;
    inputs.upperPositionRad = Units.rotationsToRadians(upperFeed.getPosition().getValueAsDouble());
    inputs.upperVelocityRadPerSec =
        Units.rotationsToRadians(upperFeed.getVelocity().getValueAsDouble());
    inputs.upperAppliedVolts = upperFeed.getMotorVoltage().getValueAsDouble();
    inputs.upperCurrentAmps = upperFeed.getSupplyCurrent().getValueAsDouble();
    inputs.upperTempCelsius = upperFeed.getDeviceTemp().getValueAsDouble();
  }

  @Override
  public void setUpperOpenLoop(double output) {
    upperFeed.set(output);
  }
}
