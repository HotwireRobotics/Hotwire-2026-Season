package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Systerface;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Logs;
import frc.robot.subsystems.ModularSubsystem;
import frc.robot.subsystems.motors.Motor;
import frc.robot.subsystems.motors.Motor.Feedback;
import frc.robot.subsystems.motors.Motor.Feedforward;
import frc.robot.subsystems.motors.MotorBase;
import frc.robot.subsystems.motors.Motor.Application;
import frc.robot.subsystems.motors.TalonFXIO;
import frc.robot.subsystems.motors.MotorBase.Direction;
import frc.robot.subsystems.motors.MotorBase.NeutralMode;
import frc.robot.subsystems.motors.MotorLogs;

import java.util.function.Supplier;

public class Shooter extends ModularSubsystem implements Systerface {

  // Abstraction
  private final ShooterBase io;
  private final ShooterLogs inputs = new ShooterLogs();

  // Declare devices.
  public final Motor feeder;
  public final Motor left;
  public final Motor right;

  // Declare device enum.
  public enum Device {
    FEEDER,
    RIGHT,
    LEFT
  }

  // Deboucner for shooter readiness.
  private final Debouncer debouncer = new Debouncer(Constants.Shooter.kDebounce.in(Seconds));

  public Shooter(ShooterBase base) {
    // Initialize abstraction.
    this.io = base;

    // Initialize devices.
    right = new Motor(this, new TalonFXIO(Constants.MotorIDs.s_shooterR));
    right.apply(
      new Application(Direction.FORWARD, NeutralMode.COAST, Amps.of(60)));

    left = new Motor(this, new TalonFXIO(Constants.MotorIDs.s_shooterL));
    left.apply(
      new Application(Direction.REVERSE, NeutralMode.COAST, Amps.of(60)));

    feeder = new Motor(this, new TalonFXIO(Constants.MotorIDs.s_feeder));
    feeder.apply(
      new Application(Direction.FORWARD, NeutralMode.COAST, Amps.of(40)));

    // Define devices.
    defineDevice(
      new DevicePointer(Device.RIGHT, right),
      new DevicePointer(Device.LEFT,  left),
      new DevicePointer(Device.FEEDER, feeder)
    );

    // Apply control constants.
    left.apply(
      new Feedforward(0.8, 0, 0), 
      new Feedback(0.24998, 0.12009, 0)
    );
    right.apply(
      new Feedforward(0.8, 0, 0), 
      new Feedback(0.34220, 0.11965, 0)
    );
    feeder.apply(
      new Feedforward(0.8, 0, 0), 
      new Feedback(0.24998, 0.12009, 0)
    );

    // var file = Filesystem.getDeployDirectory()
    //     .toPath()
    //     .resolve("shooter/config.json");

    // try {
    //     var data = new ObjectMapper().readTree(file.toFile());
    //     double rpm = data.get("rpm").asDouble();
    // } catch (IOException e) {
    //     e.printStackTrace();
    // }
  }

  private enum State {
    STOPPED,
    FIRING
  }

  private State state = State.STOPPED;

  @Override
  public Object getState() {
    return state;
  }

  public void setState(State newState) {
    state = newState;
  }

  @Override
  public void periodic() {
    logDevices();

    Logs.log(this, state);
  }

  private void applyVelocity(AngularVelocity velocity, Motor... motors) {
    for (var m : motors) m.runVelocity(velocity);
  }

  private void applyPercent(double percent, Motor... motors) {
    for (var m : motors) m.runPercent(percent);
  }

  public void start() {
    if (velocity.get().equals(RPM.of(0))) {
      applyPercent(Constants.Shooter.kZero.in(RPM), left, right, feeder);

      setState(State.STOPPED);
    } else {
      applyVelocity(velocity.get(), left, right, feeder);

      setState(State.FIRING);
    }
  }

  public void stall() {
    applyPercent(Constants.Shooter.kZero.in(RPM), left, right, feeder);

    setState(State.STOPPED);
  }

  public boolean isReady() {
    return debouncer.calculate(
        left.getVelocity().isNear(velocity.get(), Constants.Shooter.kVelocityTolerance) &&
        right.getVelocity().isNear(velocity.get(), Constants.Shooter.kVelocityTolerance));
  }

  public Command run() {
    return runOnce(() -> start());
  }

  public Command halt() {
    return runOnce(() -> stall());
  }

  public Command sysIdRightAnalysis() {
    return right.runSysId();
  }

  public Command sysIdLeftAnalysis() {
    return left.runSysId();
  }

  public Command sysIdFeederAnalysis() {
    return feeder.runSysId();
  }
}
