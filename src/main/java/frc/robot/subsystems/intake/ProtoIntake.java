package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import org.littletonrobotics.junction.Logger;

public class ProtoIntake extends ModularSubsystem implements Systerface {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum Device {
    ROLLERS
  }

  public ProtoIntake(IntakeIO io) {
    this.io = io;
  }

  private enum State {
    STOPPED,
    INTAKING
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/State", state.toString());

    if (isActiveDevice(Device.ROLLERS)) {
      state = State.INTAKING;
    } else {
      state = State.STOPPED;
    }
  }

  /** Run the rollers at the given output (-1 to 1). */
  public void runDevice(Device device, double speed) {
    if (device == Device.ROLLERS) {
      io.setRollersOutput(speed);
      if (speed == 0) {
        specifyInactiveDevice(device);
      } else {
        specifyActiveDevice(device);
      }
    }
  }

  public Command runMechanism(double speed) {
    return Commands.run(() -> runDevice(Device.ROLLERS, speed));
  }
}
