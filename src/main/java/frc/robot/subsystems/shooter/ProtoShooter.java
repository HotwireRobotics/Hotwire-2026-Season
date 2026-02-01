package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends ModularSubsystem implements Systerface {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_SysIdRoutineLeft;

  /** Which device(s) to command. Use ShooterIO.Device. */
  public enum Device {
    FEEDER,
    RIGHT,
    LEFT,
    BOTH
  }

  public ProtoShooter(ShooterIO io) {
    this.io = io;
    m_sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState/Right", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setShooterVoltage(ShooterIO.Device.RIGHT, voltage), null, this));
    m_SysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setShooterVoltage(ShooterIO.Device.LEFT, voltage), null, this));
  }

  private enum State {
    STOPPED,
    SPINNING,
    FIRING
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/State", state.toString());

    if (isActiveDevice(Device.LEFT)
        || isActiveDevice(Device.BOTH)
        || isActiveDevice(Device.RIGHT)) {
      if (isActiveDevice(Device.FEEDER)) {
        state = State.FIRING;
      } else {
        state = State.SPINNING;
      }
    } else {
      state = State.STOPPED;
    }
  }

  /** Run a device at percent output (-1 to 1). */
  public void runDevice(Device device, double speed) {
    if (device == Device.FEEDER) {
      io.setFeederOutput(speed);
    } else {
      io.setShooterOutput(ShooterIO.Device.valueOf(device.name()), speed);
    }
    if (speed == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  /** Run a device at the specified velocity. */
  public void runDeviceVelocity(Device device, AngularVelocity velocity) {
    if (device == Device.FEEDER) {
      io.setFeederVelocity(velocity);
    } else {
      io.setShooterVelocity(ShooterIO.Device.valueOf(device.name()), velocity);
    }
    if (velocity.in(RotationsPerSecond) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  /** Run a device at the specified voltage (e.g. SysId). */
  public void runDeviceVoltage(Device device, Voltage voltage) {
    io.setShooterVoltage(ShooterIO.Device.valueOf(device.name()), voltage);
    if (voltage.in(Volts) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  public Command runMechanism(double feeder, double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.BOTH, shooter);
          runDevice(Device.FEEDER, feeder);
        });
  }

  public Command runMechanismVelocity(AngularVelocity feeder, AngularVelocity shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH, shooter);
          runDeviceVelocity(Device.FEEDER, feeder);
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }
}
