package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

public class LogGroups {
    /**
     * Log TalonFX data to the Logger. This should be called periodically in the subsystem's periodic method.
     * @param motor
     */
    public static void log(TalonFX motor) {
        Logger.recordOutput("Motors/" + motor.getDeviceID() + "/Position", motor.getPosition().getValue().in(Rotations), "rotations");
        Logger.recordOutput("Motors/" + motor.getDeviceID() + "/Velocity", motor.getVelocity().getValue().in(RotationsPerSecond), "RPS");
        Logger.recordOutput("Motors/" + motor.getDeviceID() + "/Acceleration", motor.getAcceleration().getValue().in(RotationsPerSecondPerSecond), "RPS/s");
        Logger.recordOutput("Motors/" + motor.getDeviceID() + "/SupplyVoltage", motor.getSupplyVoltage().getValue().in(Volts), "V");
        Logger.recordOutput("Motors/" + motor.getDeviceID() + "/SupplyCurrent", motor.getSupplyCurrent().getValue().in(Amps), "A");
        Logger.recordOutput("Motors/" + motor.getDeviceID() + "/StatorCurrent", motor.getStatorCurrent().getValue().in(Amps), "A");
    }
}
