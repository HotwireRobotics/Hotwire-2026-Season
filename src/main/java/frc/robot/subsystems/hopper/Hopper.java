package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Logs;
import frc.robot.subsystems.ModularSubsystem;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.TalonFXIO;
import frc.robot.subsystems.motors.MotorBase.Direction;
import frc.robot.subsystems.motors.MotorBase.NeutralMode;

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
    hopper = new Motor(this, new TalonFXIO(Constants.MotorIDs.h_hopper));
    hopper.setCurrentLimit(Amps.of(60));
    hopper.setNeutralMode(NeutralMode.COAST);
    hopper.setDirection(Direction.FORWARD);

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
    return Commands.runOnce(() -> {
      hopper.runPercent(speed.get());
    });
  }

  /** Halt the hopper. */
  public Command halt() {
    return Commands.runOnce(() -> hopper.runPercent(0));
  }
}
