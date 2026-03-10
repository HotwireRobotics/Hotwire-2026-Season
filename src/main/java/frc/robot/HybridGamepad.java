package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

/**
 * Simple wrapper that auto-selects Xbox vs. DualSense mappings per port.
 *
 * <p>If the USB name contains "sense" (e.g. "DualSense Wireless Controller"), we treat it as a
 * PlayStation controller and use raw GenericHID indices based on WPILib PS4/PS5 layouts.
 */
public class HybridGamepad {
  private final CommandXboxController xbox;
  private final GenericHID dualSense;
  private final boolean usePlayStation;
  private final int port;

  public HybridGamepad(int port) {
    this.port = port;
    String name = DriverStation.getJoystickName(port).toLowerCase();
    boolean looksPlayStation = name.contains("sense");

    if (looksPlayStation) {
      dualSense = new GenericHID(port);
      xbox = null;
      usePlayStation = true;
    } else {
      xbox = new CommandXboxController(port);
      dualSense = null;
      usePlayStation = false;
    }
  }

  // Axes
  public double getLeftY() {
    if (!usePlayStation) {
      return xbox.getLeftY();
    }
    // PS5/PS4 left stick Y is axis 1
    return dualSense.getRawAxis(1);
  }

  public double getLeftX() {
    if (!usePlayStation) {
      return xbox.getLeftX();
    }
    // PS5/PS4 left stick X is axis 0
    return dualSense.getRawAxis(0);
  }

  public double getRightX() {
    if (!usePlayStation) {
      return xbox.getRightX();
    }
    // PS5/PS4 right stick X is axis 2
    return dualSense.getRawAxis(2);
  }

  // POV helpers
  public Trigger povUp() {
    if (!usePlayStation) {
      return xbox.povUp();
    }
    return new Trigger(() -> dualSense.getPOV() == 0);
  }

  public Trigger povRight() {
    if (!usePlayStation) {
      return xbox.povRight();
    }
    return new Trigger(() -> dualSense.getPOV() == 90);
  }

  public Trigger povDown() {
    if (!usePlayStation) {
      return xbox.povDown();
    }
    return new Trigger(() -> dualSense.getPOV() == 180);
  }

  // Face buttons – map Xbox A/X to Cross/Square on PS.
  public Trigger a() {
    if (!usePlayStation) {
      return xbox.a();
    }
    // PS5/PS4 Cross (bottom) is button 2
    return new Trigger(() -> dualSense.getRawButton(2));
  }

  public Trigger x() {
    if (!usePlayStation) {
      return xbox.x();
    }
    // PS5/PS4 Square (left) is button 1
    return new Trigger(() -> dualSense.getRawButton(1));
  }

  // Triggers – treat analog axis > 0.2 as "pressed"
  public Trigger leftTrigger() {
    if (!usePlayStation) {
      return xbox.leftTrigger();
    }
    // PS5/PS4 L2 axis is 3
    return new Trigger(() -> dualSense.getRawAxis(3) > 0.2);
  }

  public Trigger rightTrigger() {
    if (!usePlayStation) {
      return xbox.rightTrigger();
    }
    // PS5/PS4 R2 axis is 4
    return new Trigger(() -> dualSense.getRawAxis(4) > 0.2);
  }

  // Rumble
  public void setRumble(RumbleType type, double value) {
    if (!usePlayStation) {
      xbox.getHID().setRumble(type, value);
    } else {
      dualSense.setRumble(type, value);
    }
  }

  /** Logs axes and trigger values for debugging controller mappings. */
  public void log(String name) {
    String base = "Gamepad/" + name;

    Logger.recordOutput(base + "/Name", DriverStation.getJoystickName(port));
    Logger.recordOutput(base + "/IsPlayStation", usePlayStation);

    Logger.recordOutput(base + "/LeftX", getLeftX());
    Logger.recordOutput(base + "/LeftY", getLeftY());
    Logger.recordOutput(base + "/RightX", getRightX());

    double l2Axis;
    double r2Axis;
    if (!usePlayStation) {
      l2Axis = xbox.getLeftTriggerAxis();
      r2Axis = xbox.getRightTriggerAxis();
    } else {
      l2Axis = dualSense.getRawAxis(3);
      r2Axis = dualSense.getRawAxis(4);
    }
    Logger.recordOutput(base + "/L2Axis", l2Axis);
    Logger.recordOutput(base + "/R2Axis", r2Axis);

    int pov;
    if (!usePlayStation) {
      pov = xbox.getHID().getPOV();
    } else {
      pov = dualSense.getPOV();
    }
    Logger.recordOutput(base + "/POV", pov);
  }
}
