package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import frc.robot.subsystems.shooter.ProtoShooter.Device;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {

  private TalonFX rollers;

  public enum Device {
    ROLLERS
  }

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);

    defineDevice(Device.ROLLERS, rollers);
  }

  private enum State {
    STOPPED,
    INTAKING // Running rollers
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Intake/State", state.toString());
    Logger.recordOutput("Intake/RollersVelocity", rollers.getVelocity().getValue());
    Logger.recordOutput("Intake/RollersCurrent", rollers.getSupplyCurrent().getValue());

    if (isActiveDevice(Device.ROLLERS)) {
      state = State.INTAKING;
    } else {
      state = State.STOPPED;
    }
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

  public Command runMechanism(double speed) {
    return Commands.run(
      () -> {
        runDevice(Device.ROLLERS, speed);
      });
  }
}
