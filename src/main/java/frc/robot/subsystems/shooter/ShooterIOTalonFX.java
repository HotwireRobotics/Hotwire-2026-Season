package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX feeder = new TalonFX(Constants.MotorIDs.s_feeder);

  private final TalonFX leftShooter = new TalonFX(Constants.MotorIDs.s_shooterL);
  private final TalonFX leftFollower = new TalonFX(Constants.MotorIDs.s_followerL);
  private final TalonFX rightShooter = new TalonFX(Constants.MotorIDs.s_shooterR);
  private final TalonFX rightFollower = new TalonFX(Constants.MotorIDs.s_followerR);

  private final VoltageOut voltReq = new VoltageOut(0.0);
  private final VelocityVoltage velVolt = new VelocityVoltage(0.0);

  // Cached signals for batched refresh
  private final StatusSignal<?> feederVel, feederV, feederCurr, feederT;
  private final StatusSignal<?> leftVel, leftV, leftCurr, leftT;
  private final StatusSignal<?> rightVel, rightV, rightCurr, rightT;
  private final StatusSignal<?> leftFVel, leftFV, leftFCurr, leftFT;
  private final StatusSignal<?> rightFVel, rightFV, rightFCurr, rightFT;

  public ShooterIOTalonFX() {
    leftFollower.setControl(
        new Follower(Constants.MotorIDs.s_shooterL, MotorAlignmentValue.Opposed));
    rightFollower.setControl(
        new Follower(Constants.MotorIDs.s_shooterR, MotorAlignmentValue.Opposed));

    feederVel = feeder.getVelocity();
    feederV = feeder.getMotorVoltage();
    feederCurr = feeder.getSupplyCurrent();
    feederT = feeder.getDeviceTemp();
    leftVel = leftShooter.getVelocity();
    leftV = leftShooter.getMotorVoltage();
    leftCurr = leftShooter.getSupplyCurrent();
    leftT = leftShooter.getDeviceTemp();
    rightVel = rightShooter.getVelocity();
    rightV = rightShooter.getMotorVoltage();
    rightCurr = rightShooter.getSupplyCurrent();
    rightT = rightShooter.getDeviceTemp();
    leftFVel = leftFollower.getVelocity();
    leftFV = leftFollower.getMotorVoltage();
    leftFCurr = leftFollower.getSupplyCurrent();
    leftFT = leftFollower.getDeviceTemp();
    rightFVel = rightFollower.getVelocity();
    rightFV = rightFollower.getMotorVoltage();
    rightFCurr = rightFollower.getSupplyCurrent();
    rightFT = rightFollower.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            feederVel,
            feederV,
            feederCurr,
            feederT,
            leftVel,
            leftV,
            leftCurr,
            leftT,
            rightVel,
            rightV,
            rightCurr,
            rightT,
            leftFVel,
            leftFV,
            leftFCurr,
            leftFT,
            rightFVel,
            rightFV,
            rightFCurr,
            rightFT);

    inputs.feederConnected = status.isOK();
    inputs.feederVelocityRpm = feederVel.getValueAsDouble() * 60;
    inputs.feederVoltage = feederV.getValueAsDouble();
    inputs.feederCurrentAmps = feederCurr.getValueAsDouble();
    inputs.feederTempCelsius = feederT.getValueAsDouble();

    inputs.leftShooterConnected = status.isOK();
    inputs.leftShooterVelocityRpm = leftVel.getValueAsDouble() * 60;
    inputs.leftShooterVoltage = leftV.getValueAsDouble();
    inputs.leftShooterCurrentAmps = leftCurr.getValueAsDouble();
    inputs.leftShooterTempCelsius = leftT.getValueAsDouble();

    inputs.rightShooterConnected = status.isOK();
    inputs.rightShooterVelocityRpm = rightVel.getValueAsDouble() * 60;
    inputs.rightShooterVoltage = rightV.getValueAsDouble();
    inputs.rightShooterCurrentAmps = rightCurr.getValueAsDouble();
    inputs.rightShooterTempCelsius = rightT.getValueAsDouble();

    inputs.leftFollowerVelocityRpm = leftFVel.getValueAsDouble() * 60;
    inputs.leftFollowerVoltage = leftFV.getValueAsDouble();
    inputs.leftFollowerCurrentAmps = leftFCurr.getValueAsDouble();
    inputs.leftFollowerTempCelsius = leftFT.getValueAsDouble();

    inputs.rightFollowerVelocityRpm = rightFVel.getValueAsDouble() * 60;
    inputs.rightFollowerVoltage = rightFV.getValueAsDouble();
    inputs.rightFollowerCurrentAmps = rightFCurr.getValueAsDouble();
    inputs.rightFollowerTempCelsius = rightFT.getValueAsDouble();
  }

  @Override
  public void setFeederOutput(double output) {
    feeder.set(output);
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    feeder.setControl(velVolt.withVelocity(velocity));
  }

  @Override
  public void setShooterOutput(ShooterIO.Device device, double output) {
    switch (device) {
      case LEFT -> leftShooter.set(output);
      case RIGHT -> rightShooter.set(output);
      case BOTH -> {
        leftShooter.set(output);
        rightShooter.set(output);
      }
      default -> {}
    }
  }

  @Override
  public void setShooterVelocity(ShooterIO.Device device, AngularVelocity velocity) {
    switch (device) {
      case LEFT -> leftShooter.setControl(velVolt.withVelocity(velocity));
      case RIGHT -> rightShooter.setControl(velVolt.withVelocity(velocity));
      case BOTH -> {
        leftShooter.setControl(velVolt.withVelocity(velocity));
        rightShooter.setControl(velVolt.withVelocity(velocity));
      }
      default -> {}
    }
  }

  @Override
  public void setShooterVoltage(ShooterIO.Device device, Voltage voltage) {
    double v = voltage.in(Volts);
    switch (device) {
      case LEFT -> leftShooter.setControl(voltReq.withOutput(v));
      case RIGHT -> rightShooter.setControl(voltReq.withOutput(v));
      default -> {}
    }
  }
}
