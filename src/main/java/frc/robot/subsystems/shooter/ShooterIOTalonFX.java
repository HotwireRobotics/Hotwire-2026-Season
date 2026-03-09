package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

/** Real hardware IO for shooter; uses TalonFX hardware. */
public class ShooterIOTalonFX {
    
    private static final double SLOT0_KV = 0.11451;
    private static final double SLOT0_KS = 0.19361;
    private static final double SLOT0_KP = 0.8;

    private final TalonFX leftShooter = 
            new TalonFX(Constants.MotorIDs.s_shooterL, TunerConstants.kCANBus);
    private final TalonFX rightShooter = 
            new TalonFX(Constants.MotorIDs.s_shooterR, TunerConstants.kCANBus);
    private final TalonFX feeder = 
            new TalonFX(Constants.MotorIDs.s_feeder, TunerConstants.kCANBus);

    private final VoltageOut leftVoltageOutput = new VoltageOut(0.0);
    private final VoltageOut rightVoltageOutput = new VoltageOut(0.0);
    private final VoltageOut feederVoltageOutput = new VoltageOut(0.0);
    private final VelocityVoltage leftVelocityOutput = new VelocityVoltage(0.0);
    private final VelocityVoltage rightVelocityOutput = new VelocityVoltage(0.0);
    private final VelocityVoltage feederVelocityOutput = new VelocityVoltage(0.0);
}
