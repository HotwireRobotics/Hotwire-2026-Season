package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends ModularSubsystem implements Systerface {
  private final TalonFX feeder;
  private final SysIdRoutine m_sysIdRoutineRight;
  private final SysIdRoutine m_SysIdRoutineLeft;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;
  private final ShooterModule rightModule;
  private final ShooterModule leftModule;

  // Cached status signals for one refreshAll() per cycle (efficient CAN usage)
  private final StatusSignal<?> feederVel, feederVoltage, feederCurrent, feederTemp;
  private final StatusSignal<?> leftVel, leftVoltage, leftCurrent, leftTemp;
  private final StatusSignal<?> rightVel, rightVoltage, rightCurrent, rightTemp;
  private final StatusSignal<?> leftFollowerVel, leftFollowerVoltage, leftFollowerCurrent, leftFollowerTemp;
  private final StatusSignal<?> rightFollowerVel, rightFollowerVoltage, rightFollowerCurrent, rightFollowerTemp;

  public enum Device {
    FEEDER,
    RIGHT,
    LEFT,
    BOTH
  }

  // private final HashMap<Device, Object> devices = new HashMap<Device, Object>();
  // private List<Device> active = new ArrayList<Device>();

  private class ShooterModule {

    TalonFX shooter;
    TalonFX follower;

    public ShooterModule(int deviceID, int followerID) {
      shooter = new TalonFX(deviceID);
      follower = new TalonFX(followerID);

      follower.setControl(new Follower(deviceID, MotorAlignmentValue.Opposed));
    }

    public void runModule(double speed) {
      shooter.set(speed);
    }

    public void setControl(ControlRequest control) {
      shooter.setControl(control);
    }
  }

  public ProtoShooter() {
    feeder = new TalonFX(Constants.MotorIDs.s_feeder);

    rightModule = new ShooterModule(Constants.MotorIDs.s_shooterR, Constants.MotorIDs.s_followerR);
    leftModule = new ShooterModule(Constants.MotorIDs.s_shooterL, Constants.MotorIDs.s_followerL);

    TalonFX[] shooters = {leftModule.shooter, rightModule.shooter};

    defineDevice(Device.FEEDER, feeder);
    defineDevice(Device.RIGHT, rightModule.shooter);
    defineDevice(Device.LEFT, leftModule.shooter);
    defineDevice(Device.BOTH, shooters);

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
                (voltage) -> runDeviceVoltage(Device.RIGHT, voltage), null, this));
    m_SysIdRoutineLeft =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runDeviceVoltage(Device.LEFT, voltage), null, this));

    // Cache status signals for batched refresh (one CAN sync per cycle)
    feederVel = feeder.getVelocity();
    feederVoltage = feeder.getMotorVoltage();
    feederCurrent = feeder.getSupplyCurrent();
    feederTemp = feeder.getDeviceTemp();
    leftVel = leftModule.shooter.getVelocity();
    leftVoltage = leftModule.shooter.getMotorVoltage();
    leftCurrent = leftModule.shooter.getSupplyCurrent();
    leftTemp = leftModule.shooter.getDeviceTemp();
    rightVel = rightModule.shooter.getVelocity();
    rightVoltage = rightModule.shooter.getMotorVoltage();
    rightCurrent = rightModule.shooter.getSupplyCurrent();
    rightTemp = rightModule.shooter.getDeviceTemp();
    leftFollowerVel = leftModule.follower.getVelocity();
    leftFollowerVoltage = leftModule.follower.getMotorVoltage();
    leftFollowerCurrent = leftModule.follower.getSupplyCurrent();
    leftFollowerTemp = leftModule.follower.getDeviceTemp();
    rightFollowerVel = rightModule.follower.getVelocity();
    rightFollowerVoltage = rightModule.follower.getMotorVoltage();
    rightFollowerCurrent = rightModule.follower.getSupplyCurrent();
    rightFollowerTemp = rightModule.follower.getDeviceTemp();
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
        feederVel, feederVoltage, feederCurrent, feederTemp,
        leftVel, leftVoltage, leftCurrent, leftTemp,
        rightVel, rightVoltage, rightCurrent, rightTemp,
        leftFollowerVel, leftFollowerVoltage, leftFollowerCurrent, leftFollowerTemp,
        rightFollowerVel, rightFollowerVoltage, rightFollowerCurrent, rightFollowerTemp);

    // Log velocity (rpm), voltage, current, temp with unit metadata
    Logger.recordOutput("Shooter/Feeder/Velocity", feederVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/Feeder/Voltage", feederVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/Feeder/Current", feederCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/Feeder/Temperature", feederTemp.getValueAsDouble(), "°C");
    Logger.recordOutput("Shooter/Left/Velocity", leftVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/Left/Voltage", leftVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/Left/Current", leftCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/Left/Temperature", leftTemp.getValueAsDouble(), "°C");
    Logger.recordOutput("Shooter/Right/Velocity", rightVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/Right/Voltage", rightVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/Right/Current", rightCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/Right/Temperature", rightTemp.getValueAsDouble(), "°C");
    Logger.recordOutput("Shooter/LeftFollower/Velocity", leftFollowerVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/LeftFollower/Voltage", leftFollowerVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/LeftFollower/Current", leftFollowerCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/LeftFollower/Temperature", leftFollowerTemp.getValueAsDouble(), "°C");
    Logger.recordOutput("Shooter/RightFollower/Velocity", rightFollowerVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Shooter/RightFollower/Voltage", rightFollowerVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Shooter/RightFollower/Current", rightFollowerCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Shooter/RightFollower/Temperature", rightFollowerTemp.getValueAsDouble(), "°C");

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
