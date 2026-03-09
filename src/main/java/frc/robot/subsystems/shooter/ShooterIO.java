package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean leftShooterConnected = false;
        public double leftShooterPositionRad = 0.0;
        public double leftShooterVelocityRadPerSec = 0.0;
        public double leftShooterAppliedVolts = 0.0;
        public double leftShooterCurrentAmps = 0.0;
        public double leftShooterTemperatureC = 0.0;

        public boolean rightShooterConnected = false;
        public double rightShooterPositionRad = 0.0;
        public double rightShooterVelocityRadPerSec = 0.0;
        public double rightShooterAppliedVolts = 0.0;
        public double rightShooterCurrentAmps = 0.0;
        public double rightShooterTemperatureC = 0.0;

        public boolean feederConnected = false;
        public double feederPositionRad = 0.0;
        public double feederVelocityRadPerSec = 0.0;
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;
        public double feederTemperatureC = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    // Sets shooter and feeder open loop output
    public default void setLeftShooterOpenLoop(double output) {}

    public default void setRightShooterOpenLoop(double output) {}

    public default void setFeederOpenLoop(double output) {}
    
    // Sets shooter and feeder velocity
    public default void setLeftShooterVelocity(double velocityRadPerSec) {}

    public default void setRightShooterVelocity(double velocityRadPerSec) {}

    public default void setFeederVelocity(double velocityRadPerSec) {}

    // Sets shooter and feeder voltage
    public default void setLeftShooterVoltage(double voltage) {}

    public default void setRightShooterVoltage(double voltage) {}

    public default void setFeederVoltage(double voltage) {}

    // Sets shooter and feeder current
    public default void setLeftShooterCurrent(double current) {}

    public default void setRightShooterCurrent(double current) {}

    public default void setFeederCurrent(double current) {}

    // Sets shooter and feeder temperature
    public default void setLeftShooterTemperature(double temperature) {}

    public default void setRightShooterTemperature(double temperature) {}

    public default void setFeederTemperature(double temperature) {}

    // Sets shooter and feeder position
    public default void setLeftShooterPosition(double positionRad) {}

    public default void setRightShooterPosition(double positionRad) {}

    public default void setFeederPosition(double positionRad) {}
    
    // Sets shooter and feeder percent
    public default void setLeftShooterPercent(double percent) {}

    public default void setRightShooterPercent(double percent) {}

    public default void setFeederPercent(double percent) {}

    public default void configureSlot0Kp(double Kp) {}
}
