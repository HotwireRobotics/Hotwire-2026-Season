package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double J_KG_M2 = 0.001;
  private static final double GEAR_RATIO = 1.0;
  private static final double RAD_PER_SEC_TO_RPM = 60.0 / (2 * Math.PI);

  private final DCMotorSim feederSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, J_KG_M2, GEAR_RATIO), MOTOR);
  private final DCMotorSim leftSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, J_KG_M2, GEAR_RATIO), MOTOR);
  private final DCMotorSim rightSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(MOTOR, J_KG_M2, GEAR_RATIO), MOTOR);

  private double feederVolts = 0.0;
  private double leftVolts = 0.0;
  private double rightVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    feederSim.setInputVoltage(MathUtil.clamp(feederVolts, -12.0, 12.0));
    leftSim.setInputVoltage(MathUtil.clamp(leftVolts, -12.0, 12.0));
    rightSim.setInputVoltage(MathUtil.clamp(rightVolts, -12.0, 12.0));
    feederSim.update(0.02);
    leftSim.update(0.02);
    rightSim.update(0.02);

    inputs.feederConnected = true;
    inputs.feederVelocityRpm = feederSim.getAngularVelocityRadPerSec() * RAD_PER_SEC_TO_RPM;
    inputs.feederVoltage = feederVolts;
    inputs.feederCurrentAmps = Math.abs(feederSim.getCurrentDrawAmps());
    inputs.feederTempCelsius = 25.0;

    inputs.leftShooterConnected = true;
    inputs.leftShooterVelocityRpm = leftSim.getAngularVelocityRadPerSec() * RAD_PER_SEC_TO_RPM;
    inputs.leftShooterVoltage = leftVolts;
    inputs.leftShooterCurrentAmps = Math.abs(leftSim.getCurrentDrawAmps());
    inputs.leftShooterTempCelsius = 25.0;

    inputs.rightShooterConnected = true;
    inputs.rightShooterVelocityRpm = rightSim.getAngularVelocityRadPerSec() * RAD_PER_SEC_TO_RPM;
    inputs.rightShooterVoltage = rightVolts;
    inputs.rightShooterCurrentAmps = Math.abs(rightSim.getCurrentDrawAmps());
    inputs.rightShooterTempCelsius = 25.0;

    inputs.leftFollowerVelocityRpm = inputs.leftShooterVelocityRpm;
    inputs.leftFollowerVoltage = inputs.leftShooterVoltage;
    inputs.leftFollowerCurrentAmps = inputs.leftShooterCurrentAmps;
    inputs.leftFollowerTempCelsius = 25.0;

    inputs.rightFollowerVelocityRpm = inputs.rightShooterVelocityRpm;
    inputs.rightFollowerVoltage = inputs.rightShooterVoltage;
    inputs.rightFollowerCurrentAmps = inputs.rightShooterCurrentAmps;
    inputs.rightFollowerTempCelsius = 25.0;
  }

  @Override
  public void setFeederOutput(double output) {
    feederVolts = output * 12.0;
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    feederVolts = MathUtil.clamp(velocity.baseUnitMagnitude() * 0.5, -12.0, 12.0);
  }

  @Override
  public void setShooterOutput(ShooterIO.Device device, double output) {
    double v = output * 12.0;
    switch (device) {
      case LEFT -> leftVolts = v;
      case RIGHT -> rightVolts = v;
      case BOTH -> {
        leftVolts = v;
        rightVolts = v;
      }
      default -> {}
    }
  }

  @Override
  public void setShooterVelocity(ShooterIO.Device device, AngularVelocity velocity) {
    double v = MathUtil.clamp(velocity.baseUnitMagnitude() * 0.5, -12.0, 12.0);
    switch (device) {
      case LEFT -> leftVolts = v;
      case RIGHT -> rightVolts = v;
      case BOTH -> {
        leftVolts = v;
        rightVolts = v;
      }
      default -> {}
    }
  }

  @Override
  public void setShooterVoltage(ShooterIO.Device device, Voltage voltage) {
    double v = MathUtil.clamp(voltage.in(Volts), -12.0, 12.0);
    switch (device) {
      case LEFT -> leftVolts = v;
      case RIGHT -> rightVolts = v;
      default -> {}
    }
  }
}
