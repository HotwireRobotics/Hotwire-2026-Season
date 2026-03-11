package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HopperSubsystem extends ModularSubsystem implements Systerface {
  private final TalonFX hopper;

  public enum Device {
    HOPPER
  }

  public HopperSubsystem() {
    hopper = new TalonFX(Constants.MotorIDs.h_hopper);
    defineDevice(Device.HOPPER, hopper);
  }

  private enum State {
    STOPPED
  }

  State state = State.STOPPED;

  public Object getState() {
    return state;
  }

  @Override
  public void periodic() {
    Logger.recordOutput(
        "Hopper/upperFeed/Position", hopper.getPosition().getValueAsDouble(), "rot");
    Logger.recordOutput(
        "Hopper/upperFeed/Velocity", hopper.getVelocity().getValueAsDouble() * 60, "rpm");
    Logger.recordOutput(
        "Hopper/upperFeed/Voltage", hopper.getMotorVoltage().getValueAsDouble(), "V");
    Logger.recordOutput(
        "Hopper/upperFeed/Current", hopper.getSupplyCurrent().getValueAsDouble(), "A");
    Logger.recordOutput(
        "Hopper/upperFeed/Temperature", hopper.getDeviceTemp().getValueAsDouble(), "°C");
  }

  public Command runHopper() {
    return runDevice(Device.HOPPER, Constants.Hopper.kSpeed);
  }

  public Command runHopper(Supplier<Boolean> inverse) {
    return runDevice(Device.HOPPER, () -> (Constants.Hopper.kSpeed * (inverse.get() ? -1 : 1)));
  }

  public Command stopHopper() {
    return runDevice(Device.HOPPER, 0);
  }
}
