package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.ModularSubsystem;
import frc.robot.Systerface;
import java.util.function.Supplier;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem with left/right modules (shooter + feeder per side). Supports percent output,
 * velocity (RPS), and voltage control, plus SysId routines per side.
 */
public class ProtoShooter extends ModularSubsystem implements Systerface {

	// ----- Modules -----
	@NotNull private final ShooterModule rightModule;
	@NotNull private final ShooterModule leftModule;

	// ----- Control -----
	@NotNull private final Slot0Configs slot0Configs;
	@NotNull private final VoltageOut voltageOutRequest;
	@NotNull private final VelocityVoltage velocityVoltageRequest;

	// ----- SysId -----
	@NotNull private final SysIdRoutine sysIdRoutineRight;
	@NotNull private final SysIdRoutine sysIdRoutineLeft;

	// ----- State -----
	private enum State {
		STOPPED,
		SPINNING,
		FIRING
	}
	@NotNull private State state = State.STOPPED;

	public enum Device {
		RIGHT_FEEDER,
		RIGHT_SHOOTER,
		LEFT_FEEDER,
		LEFT_SHOOTER,
		BOTH_FEEDER,
		BOTH_SHOOTER
	}

	/** One side: shooter wheel + feeder. */
	private static final class ShooterModule {
		@NotNull private final TalonFX shooter;
		@NotNull private final TalonFX feeder;

		ShooterModule(int shooterId, int feederId) {
			shooter = new TalonFX(shooterId);
			feeder = new TalonFX(feederId);
		}

		@NotNull
		TalonFX getShooter() { return shooter; }

		@NotNull
		TalonFX getFeeder() { return feeder; }
	}

	public ProtoShooter() {
		rightModule = new ShooterModule(Constants.MotorIDs.s_shooterR, Constants.MotorIDs.s_feederR);
		leftModule = new ShooterModule(Constants.MotorIDs.s_shooterL, Constants.MotorIDs.s_feederR);

		slot0Configs = new Slot0Configs();
		slot0Configs.withKV(0.11451);
		slot0Configs.withKS(0.19361);
		slot0Configs.withKP(0.8);

		TalonFX[] shooters = { leftModule.getShooter(), rightModule.getShooter() };
		TalonFX[] feeders = { leftModule.getFeeder(), rightModule.getFeeder() };

		defineDevice(
				new DevicePointer(Device.RIGHT_FEEDER, rightModule.getFeeder()),
				new DevicePointer(Device.LEFT_FEEDER, rightModule.getFeeder()), // single feeder hardware
				new DevicePointer(Device.RIGHT_SHOOTER, rightModule.getShooter()),
				new DevicePointer(Device.LEFT_SHOOTER, leftModule.getShooter()),
				new DevicePointer(Device.BOTH_FEEDER, feeders),
				new DevicePointer(Device.BOTH_SHOOTER, shooters));

		voltageOutRequest = new VoltageOut(0.0);
		velocityVoltageRequest = new VelocityVoltage(0.0);

		SysIdRoutine.Config defaultConfig = new SysIdRoutine.Config(
				null,
				null,
				null,
				(state) -> Logger.recordOutput("Shooter/SysIdState/Right", state.toString()));
		sysIdRoutineRight = new SysIdRoutine(
				defaultConfig,
				new SysIdRoutine.Mechanism(
						(voltage) -> runDeviceVoltage(Device.RIGHT_SHOOTER, voltage), null, this));

		SysIdRoutine.Config leftConfig = new SysIdRoutine.Config(
				null,
				null,
				null,
				(state) -> Logger.recordOutput("Shooter/SysIdState/Left", state.toString()));
		sysIdRoutineLeft = new SysIdRoutine(
				leftConfig,
				new SysIdRoutine.Mechanism(
						(voltage) -> runDeviceVoltage(Device.LEFT_SHOOTER, voltage), null, this));

		applySlot0Config();
	}

	@Override
	public void periodic() {
		Logger.recordOutput("Shooter/State", state.toString());

		logMotorTelemetry("Shooter/Left", leftModule.getShooter());
		logMotorTelemetry("Shooter/Right", rightModule.getShooter());
		logMotorTelemetry("Shooter/LeftFollower", leftModule.getFeeder());
		logMotorTelemetry("Shooter/RightFollower", rightModule.getFeeder());

		updateState();
	}

	/** Log position (rot), velocity (rpm), voltage, current, temp for one motor. */
	private void logMotorTelemetry(@NotNull String prefix, @NotNull TalonFX motor) {
		Logger.recordOutput(prefix + "/Position", motor.getPosition().getValueAsDouble(), "rot");
		Logger.recordOutput(prefix + "/Velocity", motor.getVelocity().getValueAsDouble() * 60, "rpm");
		Logger.recordOutput(prefix + "/Voltage", motor.getMotorVoltage().getValueAsDouble(), "V");
		Logger.recordOutput(prefix + "/Current", motor.getSupplyCurrent().getValueAsDouble(), "A");
		Logger.recordOutput(prefix + "/Temperature", motor.getDeviceTemp().getValueAsDouble(), "°C");
	}

	private void updateState() {
		if (isActiveDevice(Device.BOTH_SHOOTER)) {
			state = isActiveDevice(Device.BOTH_FEEDER) ? State.FIRING : State.SPINNING;
		} else {
			state = State.STOPPED;
		}
	}

	@NotNull
	public Object getState() {
		return state;
	}

	// ----- Device control -----

	public void runDevice(@NotNull Device device, double speed) {
		for (TalonFX d : getDevices(device)) {
			d.set(speed);
		}
		if (speed == 0) {
			specifyInactiveDevice(device);
		} else {
			specifyActiveDevice(device);
		}
	}

	public void runDeviceVelocity(@NotNull Device device, @NotNull AngularVelocity velocity) {
		for (TalonFX d : getDevices(device)) {
			d.setControl(velocityVoltageRequest.withVelocity(velocity));
		}
		if (velocity.in(RotationsPerSecond) == 0) {
			specifyInactiveDevice(device);
		} else {
			specifyActiveDevice(device);
		}
	}

	public void runDeviceVoltage(@NotNull Device device, @NotNull Voltage voltage) {
		for (TalonFX d : getDevices(device)) {
			d.setControl(voltageOutRequest.withOutput(voltage.in(Volts)));
		}
		if (voltage.in(Volts) == 0) {
			specifyInactiveDevice(device);
		} else {
			specifyActiveDevice(device);
		}
	}

	// ----- Mechanism commands -----

	@NotNull
	public Command runMechanism(double feeder, double shooter) {
		return Commands.runOnce(() -> {
			runDevice(Device.BOTH_SHOOTER, shooter);
			runDevice(Device.BOTH_FEEDER, feeder);
		});
	}

	@NotNull
	public Command runMechanismVelocity(@NotNull AngularVelocity feeder, @NotNull AngularVelocity shooter) {
		return Commands.runOnce(() -> {
			runDeviceVelocity(Device.BOTH_SHOOTER, shooter);
			runDeviceVelocity(Device.BOTH_FEEDER, feeder);
		});
	}

	@NotNull
	public Command runMechanismVelocity(
			@NotNull Supplier<AngularVelocity> feeder, @NotNull Supplier<AngularVelocity> shooter) {
		return Commands.runOnce(() -> {
			runDeviceVelocity(Device.BOTH_SHOOTER, shooter.get());
			runDeviceVelocity(Device.BOTH_FEEDER, feeder.get());
		});
	}

	@NotNull
	public Command runRightModule(double feeder, double shooter) {
		return Commands.runOnce(() -> {
			runDevice(Device.RIGHT_SHOOTER, shooter);
			runDevice(Device.RIGHT_FEEDER, feeder);
		});
	}

	@NotNull
	public Command runLeftModule(double feeder, double shooter) {
		return Commands.runOnce(() -> {
			runDevice(Device.LEFT_SHOOTER, shooter);
			runDevice(Device.LEFT_FEEDER, feeder);
		});
	}

	@NotNull
	public Command runRightModuleVelocity(
			@NotNull Supplier<AngularVelocity> feeder, @NotNull Supplier<AngularVelocity> shooter) {
		return Commands.runOnce(() -> {
			runDeviceVelocity(Device.RIGHT_SHOOTER, shooter.get());
			runDeviceVelocity(Device.RIGHT_FEEDER, feeder.get());
		});
	}

	@NotNull
	public Command runLeftModuleVelocity(
			@NotNull Supplier<AngularVelocity> feeder, @NotNull Supplier<AngularVelocity> shooter) {
		return Commands.runOnce(() -> {
			runDeviceVelocity(Device.LEFT_SHOOTER, shooter.get());
			runDeviceVelocity(Device.LEFT_FEEDER, feeder.get());
		});
	}

	// ----- Config -----

	public void configureProportional(double Kp) {
		slot0Configs.withKP(Kp);
		applySlot0Config();
	}

	private void applySlot0Config() {
		rightModule.getShooter().getConfigurator().apply(slot0Configs);
		leftModule.getShooter().getConfigurator().apply(slot0Configs);
		rightModule.getFeeder().getConfigurator().apply(slot0Configs);
		leftModule.getFeeder().getConfigurator().apply(slot0Configs);
	}

	// ----- SysId -----

	@NotNull
	public Command sysIdQuasistaticRight(@NotNull SysIdRoutine.Direction direction) {
		return sysIdRoutineRight.quasistatic(direction);
	}

	@NotNull
	public Command sysIdDynamicRight(@NotNull SysIdRoutine.Direction direction) {
		return sysIdRoutineRight.dynamic(direction);
	}

	@NotNull
	public Command sysIdQuasistaticLeft(@NotNull SysIdRoutine.Direction direction) {
		return sysIdRoutineLeft.quasistatic(direction);
	}

	@NotNull
	public Command sysIdDynamicLeft(@NotNull SysIdRoutine.Direction direction) {
		return sysIdRoutineLeft.dynamic(direction);
	}
}
