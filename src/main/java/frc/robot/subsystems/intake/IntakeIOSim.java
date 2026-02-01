package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double GEAR_RATIO = 1.0;
  private static final double J_KG_M2 = 0.001;

  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, J_KG_M2, GEAR_RATIO), MOTOR);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.rollersConnected = true;
    inputs.rollersVelocityRpm = sim.getAngularVelocityRadPerSec() * (60.0 / (2 * Math.PI));
    inputs.rollersVoltage = appliedVolts;
    inputs.rollersCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.rollersTempCelsius = 25.0; // Sim doesn't model temp
  }

  @Override
  public void setRollersOutput(double output) {
    appliedVolts = output * 12.0;
  }
}
