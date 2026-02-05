package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import frc.robot.subsystems.shooter.ProtoShooter.Device;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {

  private final TalonFX rollers;
  private final TalonFX lower;

  public enum Device {
    ROLLERS,
    LOWER
  }

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    lower = new TalonFX(Constants.MotorIDs.i_follower);
    defineDevice(Device.ROLLERS, rollers);
    defineDevice(Device.LOWER, lower);
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

    // Log position (rot), velocity (rpm), voltage, current, temp with unit metadata
    Logger.recordOutput("Intake/Rollers/Position", rollers.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Intake/Rollers/Velocity", rollers.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Intake/Rollers/Voltage", rollers.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Intake/Rollers/Current", rollers.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Intake/Rollers/Temperature", rollers.getDeviceTemp().getValueAsDouble(), "°C");

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
          runDevice(Device.LOWER, speed);
        });
  }
}
