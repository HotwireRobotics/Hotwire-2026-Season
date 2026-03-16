package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ModularSubsystem;
import frc.robot.subsystems.Motor;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hopper extends ModularSubsystem implements Systerface {
  private final Motor hopper;
  private final Supplier<Double> speed;

  public enum Device {
    HOPPER
  }

  public Hopper(Supplier<Double> speed) {
    hopper = new Motor(this, Constants.MotorIDs.h_hopper, Amps.of(40));
    defineDevice(Device.HOPPER, hopper);
    this.speed = speed;
  }

  public Hopper() {
    this(() -> Constants.Hopper.kSpeed);
  }

  private enum State {
    STOPPED,
    FEEDING
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    hopper.log();

    if (isActiveDevice(Device.HOPPER)) {
      state = State.FEEDING;
    } else {
      state = State.STOPPED;
    }
  }

  public Command runHopper() {
    return runDevice(Device.HOPPER, speed);
  }

  public Command stopHopper() {
    Command command = runDevice(Device.HOPPER, 0, this);
    command.addRequirements(this);
    return command;
  }
}
