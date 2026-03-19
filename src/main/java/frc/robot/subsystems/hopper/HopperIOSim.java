package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of hopper IO using a single motor model. */
public class HopperIOSim implements HopperIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, 0.004, 1.0), MOTOR);
  private double appliedVolts = 0.0;

  /** Updates simulated hopper telemetry at a 20 ms loop rate. */
  @Override
  public void updateInputs(HopperIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.positionRot = Units.radiansToRotations(sim.getAngularPositionRad());
    inputs.velocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.tempC = 0.0;
  }

  /** Sets simulated open-loop command as percent output. */
  @Override
  public void setHopperOpenLoop(double output) {
    appliedVolts = output * 12.0;
  }
}
