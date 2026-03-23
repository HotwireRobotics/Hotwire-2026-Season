package frc.robot.subsystems.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Motor {

  // Abstraction
  private final MotorBase io;
  private final MotorLogs inputs = new MotorLogs();

  private final SysIdRoutine sysIdRoutine;
  private final Subsystem subsystem;

  public Motor(Subsystem subsystem, MotorBase io) {
    this.io = io;
    this.subsystem = subsystem;

    // Initialize SysId routine for this motor.
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) ->
                    Logger.recordOutput(
                        "Motors/SysId/" + this.toString(), state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.runVoltage(voltage),
                null,
                subsystem));
  }

  /** Get the enclosing subsystem for this motor. */
  public Subsystem getSubsystem() {
    return subsystem;
  }

  /**
   * Set the motor output voltage.
   *
   * @param volts
   */
  public void runVoltage(Voltage volts) {
    io.runVoltage(volts);
  }

  /**
   * Run the motor to a position.
   * 
   * @param position
   */
  public void runPosition(Angle position) {
    io.runPosition(position);
  }

  /**
   * Run the motor at a velocity.
   * 
   * @param velocity
   */
  public void runVelocity(AngularVelocity velocity) {
    io.runVelocity(velocity);
  }

  /**
   * Run percent voltage output.
   * 
   * @param percent
   */
  public void runPercent(double percent) {
    io.runPercent(percent);
  }

  /**
   * Stop the motor.
   */
  public void stop() {
    io.stop();
  }

  /** Feedforward groups. */
  public static class Feedforward {
    public double kP, kI, kD;

    public Feedforward(double kP, double kI, double kD) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
    }
  }

  /** Feedback groups. */
  public static class Feedback {
    public double kS, kV, kA;

    public Feedback(double kS, double kV, double kA) {
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }
  }

  /** Configure all gains */
  public void apply(Feedforward feedforward, Feedback feedback) {
    apply(feedback);
    apply(feedforward);
  }

  /** Apply all feedforward gains. */
  public void apply(Feedforward feedforward) {
    configureProportional(feedforward.kP);
        configureIntegral(feedforward.kI);
      configureDerivative(feedforward.kD);
  }

  /** Apply all feedback gains. */
  public void apply(Feedback feedback) {
     configureStaticFriction(feedback.kS);
           configureVelocity(feedback.kV);
       configureAcceleration(feedback.kA);
  }

  /**
   * Configure proportional gain for this motor.
   * 
   * @param kP
   */
  public void configureProportional(double kP) {
    io.configureProportional(kP);
  }

  /**
   * Configure integral gain for this motor.
   * 
   * @param kI
   */
  public void configureIntegral(double kI) {
    io.configureIntegral(kI);
  }

  /**
   * Configure derivative gain for this motor.
   * 
   * @param kD
   */
  public void configureDerivative(double kD) {
    io.configureDerivative(kD);
  }

  /**
   * Configure static friction feedforward for this motor.
   * 
   * @param kS
   */
  public void configureStaticFriction(double kS) {
    io.configureStaticFriction(kS);
  }

  /**
   * Configure velocity feedforward for this motor.
   * 
   * @param kV
   */
  public void configureVelocity(double kV) {
    io.configureVelocity(kV);
  }

  /**
   * Configure acceleration feedforward for this motor.
   * 
   * @param kA
   */
  public void configureAcceleration(double kA) {
    io.configureAcceleration(kA);
  }

  /** Configuration groups. */
  public static class Application {
    public MotorBase.Direction direction;
    public MotorBase.NeutralMode neutral;
    public Current limit;

    public Application(
        MotorBase.Direction direction, MotorBase.NeutralMode neutral, Current limit) {
      this.direction = direction;
      this.neutral = neutral;
      this.limit = limit;
    }
  }

  /**
   * Apply a configuration.
   * 
   * @param application
   */
  public void apply(Application application) {
    setDirection(application.direction);
    setNeutralMode(application.neutral);
    setCurrentLimit(application.limit);
  }

  /**
   * Set the current limit for this motor.
   *
   * @param limit
   */
  public void setCurrentLimit(Current limit) {
    io.setCurrentLimit(limit);
  }

  /** Set the direction for this motor. */
  public void setDirection(MotorBase.Direction direction) {
    io.setDirection(direction);
  }

  /** Set the neutral mode for this motor. */
  public void setNeutralMode(MotorBase.NeutralMode mode) {
    io.setNeutralMode(mode);
  }
  
  /** Set this motor to follow a leader motor. */
  public void follow(Motor leader, MotorBase.FollowerMode mode) {
    io.follow(leader.getID(), mode);
  }

  /** Set this motor to follow a leader motor by id. */
  public void follow(int leader, MotorBase.FollowerMode mode) {
    io.follow(leader, mode);
  }

  /**
   * Get the current velocity of the motor.
   * 
   * @return velocity
   */
  public AngularVelocity getVelocity() {
    return io.getVelocity();
  }

  /**
   * Get the current position of the motor.
   * 
   * @return position
   */
  public Angle getPosition() {
    return io.getPosition();
  }

  /**
   * Get the current drawn by the motor.
   * 
   * @return current
   */
  public Current getCurrent() {
    return io.getCurrent();
  }

  /**
   * Get device id.
   */
  public int getID() {
    return io.getID();
  }

  /**
   * Log motor data to the Logger. This should be called periodically in the subsystem's periodic
   * method.
   */
  public void log() {
    io.updateInputs(inputs);

    Logger.recordOutput("Motor/Velocity", inputs.velocity);
    Logger.recordOutput("Motor/Current", inputs.current);
    Logger.recordOutput("Motor/Voltage", inputs.voltage);
  }

  /**
   * Run the quasistatic SysId routine for this motor.
   *
   * @param direction
   */
  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Run the dynamic SysId routine for this motor.
   *
   * @param direction
   */
  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /**
   * Run all SysId routines in sequence for this motor, including both quasistatic and dynamic
   * tests in both directions.
   */
  public Command runSysId() {
    return new SequentialCommandGroup(
        sysIdQuasistatic(SysIdRoutine.Direction.kForward),
        sysIdQuasistatic(SysIdRoutine.Direction.kReverse),
        sysIdDynamic(SysIdRoutine.Direction.kForward),
        sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}