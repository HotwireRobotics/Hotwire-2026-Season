package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Logs;
import frc.robot.subsystems.ModularSubsystem;
import frc.robot.subsystems.motors.Motor;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

public class Hopper extends ModularSubsystem implements Systerface {
  private final Motor hopper;

  // Declare suppliers.
  private final Supplier<Double> speed;

  // Declare device enum.
  public enum Device {
    HOPPER
  }

  public Hopper(Supplier<Double> speed) {
    // Initialize devices.
    hopper = new Motor(this, Constants.MotorIDs.h_hopper, Amps.of(40));
    hopper.setDirection(InvertedValue.Clockwise_Positive, NeutralModeValue.Coast);

    // Define devices.
    defineDevice(Device.HOPPER, hopper);

    // Intialize suppliers.
    this.speed = speed;
  }

  public Hopper() {
    this(() -> Constants.Hopper.kSpeed);
  }

  // State system.
  private enum State {
    STOPPED,
    FEEDING
  }

  State state = State.STOPPED;

  // Supply state.
  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    // Log devices and state.
    logDevices();
    
    Logs.log(this, state);

    if (isActiveDevice(Device.HOPPER)) {
      state = State.FEEDING;
    } else {
      state = State.STOPPED;
    }
  }

  /** Run the hopper at the specified speed. */
  public Command run() {
    return runDevice(Device.HOPPER, speed, this);
  }

  /** Halt the hopper. */
  public Command halt() {
    return runDevice(Device.HOPPER, 0, this);
  }
}
