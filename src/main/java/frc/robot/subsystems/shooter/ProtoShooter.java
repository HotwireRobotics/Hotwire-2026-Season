package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Dictionary;
import java.util.HashMap;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends SubsystemBase implements Systerface {
  private final TalonFX feeder;
  private final SysIdRoutine m_sysIdRoutine;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;
  private final ShooterModule rightModule;
  private final ShooterModule leftModule;

  public enum Device {
    FEEDER, RIGHT, LEFT, BOTH
  }

  private final HashMap<Device, Object> devices = new HashMap<Device, Object>();

  private class ShooterModule {

    TalonFX shooter;
    TalonFX follower;

    public ShooterModule(int deviceID, int followerID) {
      shooter =  new TalonFX(deviceID);
      follower = new TalonFX(followerID);

      follower.setControl(
        new Follower(
          deviceID, MotorAlignmentValue.Opposed
        ));
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

    rightModule = new ShooterModule(
      Constants.MotorIDs.s_shooterR,
      Constants.MotorIDs.s_followerR
    );

    leftModule = new ShooterModule(
      Constants.MotorIDs.s_shooterL,
      Constants.MotorIDs.s_followerL
    );

    TalonFX[] shooters = {leftModule.shooter, rightModule.shooter};

    devices.put(Device.FEEDER,feeder);
    devices.put(Device.RIGHT, rightModule.shooter);
    devices.put(Device.LEFT,  leftModule.shooter);
    devices.put(Device.BOTH,  shooters);


    m_voltReq = new VoltageOut(0.0);
    m_velVolt = new VelocityVoltage(0.0);

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runShooterVolts(voltage),
                null, // No log consumer, since data is recorded *buh* AdvantageKit
                this));
  }

  private enum State {
    STOPPED,
    HALTED,
    SPINNING, // Running shooter
    FIRING // Running shooter & feeder
  }

  State state = State.STOPPED;

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/State", state.toString());
    // Logger.recordOutput("Shooter/Feeder/Velocity", feeder.getVelocity().getValue());
    // Logger.recordOutput("Shooter/Shooter/Velocity", shooter.getVelocity().getValue());
    // Logger.recordOutput("Shooter/Follower/Velocity", follower.getVelocity().getValue());
    // Logger.recordOutput("Shooter/Feeder/Current", feeder.getSupplyCurrent().getValue());
    // Logger.recordOutput("Shooter/Shooter/Current", shooter.getSupplyCurrent().getValue());
    // Logger.recordOutput("Shooter/Follower/Current", follower.getSupplyCurrent().getValue());
  }

  public Object getState() {
    return state;
  }

  public Command runMechanism(double speed) {
    return Commands.runOnce(
        () -> {
          rightModule.runModule(speed);
          leftModule .runModule(speed);
          feeder.set(0);
        });
  }

  public Command runDevice(Device device, double speed) {
    Object group = devices.get(device);
    return Commands.runOnce((group instanceof TalonFX[]) ? () -> {
        for (TalonFX motor : (TalonFX[]) group) {
          motor.set(speed);
        }
    } : () -> ((TalonFX) group).set(speed));
  }

  public void runShooterVel(AngularVelocity velocity) {
    rightModule.setControl(m_velVolt.withVelocity(velocity));
    leftModule .setControl(m_velVolt.withVelocity(velocity));
  }

  public void runFeederVel(AngularVelocity velocity) {
    feeder.setControl(m_velVolt.withVelocity(velocity));
  }

  public void runShooterVolts(Voltage voltage) {
    rightModule.setControl(m_voltReq.withOutput(voltage.in(Volts)));
    leftModule .setControl(m_voltReq.withOutput(voltage.in(Volts)));
  }

  public Command runMechanismVelocity(AngularVelocity feeder, AngularVelocity shooter) {
    return Commands.runOnce(
      () -> {
        runShooterVel(shooter);
        runFeederVel (feeder);
      });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
