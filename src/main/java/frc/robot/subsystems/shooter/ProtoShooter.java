package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends ModularSubsystem implements Systerface {
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_sysIdRoutineLeft;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;
  private final ShooterModule rightModule;
  private final ShooterModule leftModule;

  // Cached status signals for one refreshAll() per cycle (efficient CAN usage)
  private final StatusSignal<?> leftPos, leftVel, leftVoltage, leftCurrent, leftTemp;
  private final StatusSignal<?> rightPos, rightVel, rightVoltage, rightCurrent, rightTemp;
  private final StatusSignal<?> leftFollowerPos,
      leftFollowerVel,
      leftFollowerVoltage,
      leftFollowerCurrent,
      leftFollowerTemp;
  private final StatusSignal<?> rightFollowerPos,
      rightFollowerVel,
      rightFollowerVoltage,
      rightFollowerCurrent,
      rightFollowerTemp;

  public enum Device {
    RIGHT_FEEDER,
    RIGHT_SHOOTER,
    LEFT_FEEDER,
    LEFT_SHOOTER,
    BOTH_FEEDER,
    BOTH_SHOOTER
  }

  // private final HashMap<Device, Object> devices = new HashMap<Device, Object>();
  // private List<Device> active = new ArrayList<Device>();

  private class ShooterModule {

    TalonFX shooter;
    TalonFX feeder;

    public ShooterModule(int deviceID, int followerID) {
      shooter = new TalonFX(deviceID);
      feeder = new TalonFX(followerID);
    }

    public void runModule(double speed) {
      shooter.set(speed);
    }

    public void setControl(ControlRequest control) {
      shooter.setControl(control);
    }
  }

  public ProtoShooter() {

    rightModule = new ShooterModule(Constants.MotorIDs.s_shooterR, Constants.MotorIDs.s_feederR);
    leftModule = new ShooterModule(Constants.MotorIDs.s_shooterL, Constants.MotorIDs.s_feederL);

    final TalonFX[] shooters = {leftModule.shooter, rightModule.shooter};
    final TalonFX[] feeders = {leftModule.feeder, rightModule.feeder};

    defineDevice(Device.RIGHT_FEEDER, rightModule.feeder);
    defineDevice(Device.LEFT_FEEDER, rightModule.feeder);
    defineDevice(Device.RIGHT_SHOOTER, rightModule.shooter);
    defineDevice(Device.LEFT_SHOOTER, leftModule.shooter);
    defineDevice(Device.BOTH_FEEDER, feeders);
    defineDevice(Device.BOTH_SHOOTER, shooters);

    m_voltReq = new VoltageOut(0.0);
    m_velVolt = new VelocityVoltage(0.0);

    m_sysIdRoutineRight =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/SysIdState/Right", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.RIGHT_SHOOTER, voltage), null, this));
    m_sysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.LEFT_SHOOTER, voltage), null, this));

    // Cache status signals for batched refresh (one CAN sync per cycle)
    leftPos = leftModule.shooter.getPosition();
    leftVel = leftModule.shooter.getVelocity();
    leftVoltage = leftModule.shooter.getMotorVoltage();
    leftCurrent = leftModule.shooter.getSupplyCurrent();
    leftTemp = leftModule.shooter.getDeviceTemp();
    rightPos = rightModule.shooter.getPosition();
    rightVel = rightModule.shooter.getVelocity();
    rightVoltage = rightModule.shooter.getMotorVoltage();
    rightCurrent = rightModule.shooter.getSupplyCurrent();
    rightTemp = rightModule.shooter.getDeviceTemp();
    leftFollowerPos = leftModule.feeder.getPosition();
    leftFollowerVel = leftModule.feeder.getVelocity();
    leftFollowerVoltage = leftModule.feeder.getMotorVoltage();
    leftFollowerCurrent = leftModule.feeder.getSupplyCurrent();
    leftFollowerTemp = leftModule.feeder.getDeviceTemp();
    rightFollowerPos = rightModule.feeder.getPosition();
    rightFollowerVel = rightModule.feeder.getVelocity();
    rightFollowerVoltage = rightModule.feeder.getMotorVoltage();
    rightFollowerCurrent = rightModule.feeder.getSupplyCurrent();
    rightFollowerTemp = rightModule.feeder.getDeviceTemp();
  }

  private enum State {
    STOPPED,
    SPINNING, // Running shooter
    FIRING // Running shooter & feeder
  }

  State state = State.STOPPED;

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/State", state.toString());

    // One batched CAN refresh per cycle, then read cached values (efficient)
    BaseStatusSignal.refreshAll(
        leftPos,
        leftVel,
        leftVoltage,
        leftCurrent,
        leftTemp,
        rightPos,
        rightVel,
        rightVoltage,
        rightCurrent,
        rightTemp,
        leftFollowerPos,
        leftFollowerVel,
        leftFollowerVoltage,
        leftFollowerCurrent,
        leftFollowerTemp,
        rightFollowerPos,
        rightFollowerVel,
        rightFollowerVoltage,
        rightFollowerCurrent,
        rightFollowerTemp);

    // Log position (rot), velocity (rpm), voltage, current, temp with unit metadata
    Logger.recordOutput("Shooter/Left/Position", leftPos.getValueAsDouble(), "rot");
    Logger.recordOutput("Shooter/Left/Velocity", leftVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/Left/Voltage", leftVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/Left/Current", leftCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/Left/Temperature", leftTemp.getValueAsDouble(), "°C");
    Logger.recordOutput("Shooter/Right/Position", rightPos.getValueAsDouble(), "rot");
    Logger.recordOutput("Shooter/Right/Velocity", rightVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/Right/Voltage", rightVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/Right/Current", rightCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/Right/Temperature", rightTemp.getValueAsDouble(), "°C");
    Logger.recordOutput(
        "Shooter/LeftFollower/Position", leftFollowerPos.getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Shooter/LeftFollower/Velocity", leftFollowerVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Shooter/LeftFollower/Voltage", leftFollowerVoltage.getValueAsDouble(), "V");
    Logger.recordOutput(
        "Shooter/LeftFollower/Current", leftFollowerCurrent.getValueAsDouble(), "A");
    Logger.recordOutput(
        "Shooter/LeftFollower/Temperature", leftFollowerTemp.getValueAsDouble(), "°C");
    Logger.recordOutput(
        "Shooter/RightFollower/Position", rightFollowerPos.getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Shooter/RightFollower/Velocity", rightFollowerVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Shooter/RightFollower/Voltage", rightFollowerVoltage.getValueAsDouble(), "V");
    Logger.recordOutput(
        "Shooter/RightFollower/Current", rightFollowerCurrent.getValueAsDouble(), "A");
    Logger.recordOutput(
        "Shooter/RightFollower/Temperature", rightFollowerTemp.getValueAsDouble(), "°C");

    if (isActiveDevice(Device.BOTH_SHOOTER)) {
      if (isActiveDevice(Device.BOTH_FEEDER)) {
        state = State.FIRING;
      } else {
        state = State.SPINNING;
      }
    } else {
      state = State.STOPPED;
    }
  }

  public Object getState() {
    return state;
  }

  // Device control methods
  public void runDevice(Device device, double speed) {
    for (TalonFX d : getDevices(device)) {
      d.set(speed);
    }

    if (speed == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  public void runDeviceVelocity(Device device, AngularVelocity velocity) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_velVolt.withVelocity(velocity));
    }

    if (velocity.in(RotationsPerSecond) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  public void runDeviceVoltage(Device device, Voltage voltage) {
    for (TalonFX d : getDevices(device)) {
      d.setControl(m_voltReq.withOutput(voltage.in(Volts)));
    }

    if (voltage.in(Volts) == 0) {
      specifyInactiveDevice(device);
    } else {
      specifyActiveDevice(device);
    }
  }

  // Mechanism control commands
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

  public Command sysIdQuasistaticRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.quasistatic(direction);
  }

  public Command sysIdDynamicRight(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineRight.dynamic(direction);
  }

  public Command sysIdQuasistaticLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.quasistatic(direction);
  }

  public Command sysIdDynamicLeft(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineLeft.dynamic(direction);
  }
}
