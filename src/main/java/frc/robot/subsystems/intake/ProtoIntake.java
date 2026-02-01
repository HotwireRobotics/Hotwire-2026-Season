package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
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
  // Cached status signals for one refreshAll() per cycle (efficient CAN usage)
  private final StatusSignal<?> rollersVel, rollersVoltage, rollersCurrent, rollersTemp;

  public enum Device {
    ROLLERS
  }

  public ProtoIntake() {
    rollers = new TalonFX(Constants.MotorIDs.i_rollers);
    defineDevice(Device.ROLLERS, rollers);

    // Cache status signals for batched refresh (one CAN sync per cycle)
    rollersVel = rollers.getVelocity();
    rollersVoltage = rollers.getMotorVoltage();
    rollersCurrent = rollers.getSupplyCurrent();
    rollersTemp = rollers.getDeviceTemp();
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

    // One batched CAN refresh per cycle, then read cached values (efficient)
    BaseStatusSignal.refreshAll(rollersVel, rollersVoltage, rollersCurrent, rollersTemp);

    // Log velocity (rpm), voltage, current, temp with unit metadata
    Logger.recordOutput("Intake/Rollers/Velocity", rollersVel.getValueAsDouble() * 60, "rpm");
    Logger.recordOutput("Intake/Rollers/Voltage", rollersVoltage.getValueAsDouble(), "V");
    Logger.recordOutput("Intake/Rollers/Current", rollersCurrent.getValueAsDouble(), "A");
    Logger.recordOutput("Intake/Rollers/Temperature", rollersTemp.getValueAsDouble(), "°C");

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
