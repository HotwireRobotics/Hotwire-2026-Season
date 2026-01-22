package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoShooter extends SubsystemBase implements Systerface {
  public TalonFX feeder;
  public TalonFX shooter;
  public TalonFX follower;

  public ProtoShooter() {
    feeder = new TalonFX(Constants.MotorIDs.s_feeder);
    shooter = new TalonFX(Constants.MotorIDs.s_shooter);
    follower = new TalonFX(Constants.MotorIDs.s_follower);

    // Follower motor for the shooter.
    follower.setControl(new Follower(Constants.MotorIDs.s_shooter, MotorAlignmentValue.Opposed));
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
}
