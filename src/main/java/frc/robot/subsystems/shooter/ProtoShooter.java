package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends SubsystemBase implements Systerface {
  public TalonFX feeder;
  public TalonFX shooter;
  public TalonFX follower;
  private final SysIdRoutine m_sysIdRoutine;
  private final VoltageOut m_voltReq;
  private final VelocityVoltage m_velVolt;

  public ProtoShooter() {
    feeder = new TalonFX(Constants.MotorIDs.s_feeder);
    shooter = new TalonFX(Constants.MotorIDs.s_shooter);
    follower = new TalonFX(Constants.MotorIDs.s_follower);
    m_voltReq = new VoltageOut(0.0);
    m_velVolt = new VelocityVoltage(0.0);

    // Follower motor for the shooter.
    follower.setControl(new Follower(Constants.MotorIDs.s_shooter, MotorAlignmentValue.Opposed));

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("Shooter/SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runShooterVolts(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  private enum State {
    STOPPED,
    HALTED,
    SPINNING, // Running shooter
    SHOOTING // Running shooter & feeder
  }

  State state = State.STOPPED;

  @Override
  public void periodic() {
    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/Feeder/Velocity", feeder.getVelocity().getValue());
    Logger.recordOutput("Shooter/Shooter/Velocity", shooter.getVelocity().getValue());
    Logger.recordOutput("Shooter/Follower/Velocity", follower.getVelocity().getValue());
    Logger.recordOutput("Shooter/Feeder/Current", feeder.getSupplyCurrent().getValue());
    Logger.recordOutput("Shooter/Shooter/Current", shooter.getSupplyCurrent().getValue());
    Logger.recordOutput("Shooter/Follower/Current", follower.getSupplyCurrent().getValue());
  }

  public Object getState() {
    return state;
  }

  public Command runShooter(double speed) {
    return Commands.runOnce(
        () -> {
          shooter.set(speed);
          feeder.set(0);
          if (speed > 0) {
            state = State.SPINNING;
          } else {
            state = State.STOPPED;
          }
        });
  }

  public void runShooterVel(AngularVelocity velocity) {
    shooter.setControl(m_velVolt.withVelocity(velocity));
    if (velocity.in(RotationsPerSecond) == 0.0) {
      state = State.STOPPED;
    } else {
      state = State.SPINNING;
    }
    ;
  }

  public void runFeederVel(AngularVelocity velocity) {
    feeder.setControl(m_velVolt.withVelocity(velocity));
    if (velocity.in(RotationsPerSecond) == 0.0) {
      state = State.STOPPED;
    } else {
      state = State.SPINNING;
    }
    ;
  }

  public void runShooterVolts(Voltage voltage) {
    shooter.setControl(m_voltReq.withOutput(voltage.in(Volts)));
    if (voltage.in(Volts) == 0.0) {
      state = State.STOPPED;
    } else {
      state = State.SPINNING;
    }
    ;
  }

  public Command runShooterAndFeeder(AngularVelocity velocity) {
    return Commands.runOnce(
        () -> {
          runShooterVel(velocity);
          runFeederVel(velocity);
        });
  }

  public Command runShooterAndFeeder(double speed) {
    return Commands.runOnce(
        () -> {
          shooter.set(speed);
          feeder.set(speed);
          if (speed > 0) {
            state = State.SHOOTING;
          } else {
            state = State.STOPPED;
          }
        });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
