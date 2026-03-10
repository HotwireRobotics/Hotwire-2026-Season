package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Simple wrapper that auto-selects Xbox vs. PlayStation mappings per port. */
public class HybridGamepad {
  private final CommandXboxController xbox;
  private final CommandPS4Controller ps;
  private final boolean usePs;

  public HybridGamepad(int port) {
    String name = DriverStation.getJoystickName(port).toLowerCase();
    // Common DualShock/DualSense names on Windows/DS: "wireless controller", "dualsense"
    boolean looksPlayStation =
        name.contains("wireless controller")
            || name.contains("dualsense")
            || name.contains("dualshock")
            || name.contains("playstation")
            || name.contains("ps5")
            || name.contains("ps4");

    if (looksPlayStation) {
      ps = new CommandPS4Controller(port);
      xbox = null;
      usePs = true;
    } else {
      xbox = new CommandXboxController(port);
      ps = null;
      usePs = false;
    }
  }

  // Axes
  public double getLeftY() {
    return usePs ? ps.getLeftY() : xbox.getLeftY();
  }

  public double getLeftX() {
    return usePs ? ps.getLeftX() : xbox.getLeftX();
  }

  public double getRightX() {
    return usePs ? ps.getRightX() : xbox.getRightX();
  }

  // POV helpers
  public Trigger povUp() {
    return usePs ? ps.povUp() : xbox.povUp();
  }

  public Trigger povRight() {
    return usePs ? ps.povRight() : xbox.povRight();
  }

  public Trigger povDown() {
    return usePs ? ps.povDown() : xbox.povDown();
  }

  // Face buttons – map Xbox A/X to Cross/Square on PS.
  public Trigger a() {
    return usePs ? ps.cross() : xbox.a();
  }

  public Trigger x() {
    return usePs ? ps.square() : xbox.x();
  }

  // Triggers
  public Trigger leftTrigger() {
    return usePs ? ps.L2() : xbox.leftTrigger();
  }

  public Trigger rightTrigger() {
    return usePs ? ps.R2() : xbox.rightTrigger();
  }

  // Rumble
  public void setRumble(RumbleType type, double value) {
    if (usePs) {
      ps.getHID().setRumble(type, value);
    } else {
      xbox.getHID().setRumble(type, value);
    }
  }
}
