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

  public static enum Direction {
    FORWARD,
    REVERSE
  }
  /** Set direction. */
  void setDirection(Direction direction);

  public static enum NeutralMode {
    COAST,
    BRAKE
  }
  /** Set neutral mode. */
  void setNeutralMode(NeutralMode mode);

  public static enum FollowerMode {
    ALIGNED,
    INVERSE
  }
  /** Set motor follower target. */
  void follow(Motor leader, FollowerMode mode);
  /** Set motor follower target from id. */
  void follow(int leader, FollowerMode mode);

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