package frc.robot.subsystems.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface MotorBase {

  /** Set output voltage */
  void runVoltage(Voltage volts);

  /** Stop motor */
  void stop();

  /** Update sensor inputs (for logging) */
  void updateInputs(MotorLogs inputs);

  /** Limit current. */
  void setCurrentLimit(Current limit);

  /** Set direction. */
  public static enum Direction {
    FORWARD,
    REVERSE
  }
  void setDirection(Direction direction);

  /** Set neutral mode. */
  public static enum NeutralMode {
    COAST,
    BRAKE
  }
  void setNeutralMode(NeutralMode mode);

  // Configure feedback and feedforward values.
  void configureProportional(double kP);

  void configureIntegral(double kI);

  void configureDerivative(double kD);

  void configureStaticFriction(double kS);

  void configureVelocity(double kV);

  void configureAcceleration(double kA);

  /** Run to position. */
  void runPosition(Angle position);

  /** Run to velocity. */
  void runVelocity(AngularVelocity velocity);

  /** Run to percent. */
  void runPercent(double percent);

  /** Get measured velocity. */
  AngularVelocity getVelocity();

  /** Get measured position. */
  Angle getPosition();

  /** Get measured current. */
  Current getCurrent();

  /** Get device id. */
  int getID();
}