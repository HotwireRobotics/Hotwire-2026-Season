package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Systerface;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem. All hardware interaction is via ShooterIO so inputs are loggable and
 * replayable. Three motors: left shooter, right shooter, feeder (original had shared feeder).
 */
public class ProtoShooter extends SubsystemBase implements Systerface {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Last commanded outputs for state derivation (no hardware in subsystem). */
  private double lastLeftShooter = 0.0;

  private double lastRightShooter = 0.0;
  private double lastFeeder = 0.0;

  public enum Device {
    RIGHT_FEEDER,
    RIGHT_SHOOTER,
    LEFT_FEEDER,
    LEFT_SHOOTER,
    BOTH_FEEDER,
    BOTH_SHOOTER
  }

  private final SysIdRoutine sysIdRight;
  private final SysIdRoutine sysIdLeft;

  public ProtoShooter(ShooterIO io) {
    this.io = io;
    sysIdRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState/Right", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setRightShooterVoltage(voltage.in(Volts)), null, this));
    sysIdLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setLeftShooterVoltage(voltage.in(Volts)), null, this));
  }

  private enum State {
    STOPPED,
    SPINNING,
    FIRING
  }

  State state = State.STOPPED;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/State", state.toString());

    boolean shooterActive = lastLeftShooter != 0.0 || lastRightShooter != 0.0;
    boolean feederActive = lastFeeder != 0.0;
    if (shooterActive && feederActive) {
      state = State.FIRING;
    } else if (shooterActive) {
      state = State.SPINNING;
    } else {
      state = State.STOPPED;
    }
  }

  @Override
  public Object getState() {
    return state;
  }

  public void runDevice(Device device, double speed) {
    switch (device) {
      case LEFT_SHOOTER -> {
        io.setLeftShooterPercent(speed);
        lastLeftShooter = speed;
      }
      case RIGHT_SHOOTER -> {
        io.setRightShooterPercent(speed);
        lastRightShooter = speed;
      }
      case LEFT_FEEDER, RIGHT_FEEDER, BOTH_FEEDER -> {
        io.setFeederPercent(speed);
        lastFeeder = speed;
      }
      case BOTH_SHOOTER -> {
        io.setLeftShooterPercent(speed);
        io.setRightShooterPercent(speed);
        lastLeftShooter = speed;
        lastRightShooter = speed;
      }
    }
  }

  public void runDeviceVelocity(Device device, AngularVelocity velocity) {
    double rps = velocity.in(RotationsPerSecond);
    switch (device) {
      case LEFT_SHOOTER -> {
        io.setLeftShooterVelocityRotPerSec(rps);
        lastLeftShooter = rps;
      }
      case RIGHT_SHOOTER -> {
        io.setRightShooterVelocityRotPerSec(rps);
        lastRightShooter = rps;
      }
      case LEFT_FEEDER, RIGHT_FEEDER, BOTH_FEEDER -> {
        io.setFeederVelocityRotPerSec(rps);
        lastFeeder = rps;
      }
      case BOTH_SHOOTER -> {
        io.setLeftShooterVelocityRotPerSec(rps);
        io.setRightShooterVelocityRotPerSec(rps);
        lastLeftShooter = rps;
        lastRightShooter = rps;
      }
    }
  }

  public void runDeviceVoltage(Device device, Voltage voltage) {
    double v = voltage.in(Volts);
    switch (device) {
      case LEFT_SHOOTER -> io.setLeftShooterVoltage(v);
      case RIGHT_SHOOTER -> io.setRightShooterVoltage(v);
      case LEFT_FEEDER, RIGHT_FEEDER, BOTH_FEEDER -> io.setFeederVoltage(v);
      case BOTH_SHOOTER -> {
        io.setLeftShooterVoltage(v);
        io.setRightShooterVoltage(v);
      }
    }
  }

  public Command runMechanism(double feeder, double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.BOTH_SHOOTER, shooter);
          runDevice(Device.BOTH_FEEDER, feeder);
        });
  }

  public Command runMechanismVelocity(AngularVelocity feeder, AngularVelocity shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooter);
          runDeviceVelocity(Device.BOTH_FEEDER, feeder);
        });
  }

  public Command runMechanismVelocity(
      Supplier<AngularVelocity> feeder, Supplier<AngularVelocity> shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.BOTH_SHOOTER, shooter.get());
          runDeviceVelocity(Device.BOTH_FEEDER, feeder.get());
        });
  }

  public Command runRightModule(double feeder, double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.RIGHT_SHOOTER, shooter);
          runDevice(Device.RIGHT_FEEDER, feeder);
        });
  }

  public Command runLeftModule(double feeder, double shooter) {
    return Commands.runOnce(
        () -> {
          runDevice(Device.LEFT_SHOOTER, shooter);
          runDevice(Device.LEFT_FEEDER, feeder);
        });
  }

  public Command runRightModuleVelocity(
      Supplier<AngularVelocity> feeder, Supplier<AngularVelocity> shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.RIGHT_SHOOTER, shooter.get());
          runDeviceVelocity(Device.RIGHT_FEEDER, feeder.get());
        });
  }

  public Command runLeftModuleVelocity(
      Supplier<AngularVelocity> feeder, Supplier<AngularVelocity> shooter) {
    return Commands.runOnce(
        () -> {
          runDeviceVelocity(Device.LEFT_SHOOTER, shooter.get());
          runDeviceVelocity(Device.LEFT_FEEDER, feeder.get());
        });
  }

  public void configureProportional(double kp) {
    io.configureSlot0Kp(kp);
  }

  public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
    return sysIdRight.quasistatic(direction);
  }

  public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
    return sysIdRight.dynamic(direction);
  }

  public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
    return sysIdLeft.quasistatic(direction);
  }

  public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
    return sysIdLeft.dynamic(direction);
  }
}
