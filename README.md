# 2026 Hotwire

This repository is **Hotwire Robotics (2990)**'s robot code for the 2026 game, **Rebuilt!** 🟡

[![Hotwire Robotics Logo](assets/hotwire.svg)](https://hotwirerobotics.com)

---

## Overview

**Ro-bert** is Hotwire’s 2026 competition bot: trench runner, dual-barrel shooter, and L3 climber.

### Physical

- **Drive** — Swerve; under the trench (with option to go over the bump)
- **Shooter** — Fixed, fast dual-barrel flywheel (no hood)
- **Climb** — L3 capable
- **Capacity** — As much fuel as possible while still fitting under the trench

### Programming

- **On-the-move shooting** with distance-scaled RPM
- **Multi-auto** — different autos for each situation
- **Multi-Limelight** robot positioning (AprilTags)
- **Feeder station** intake in addition to ground / over-bumper

---

## Robot Architecture

| Subsystem | Description |
| --------- | ----------- |
| **Drive** | Swerve drive (4× Talon FX, Pigeon 2), PathPlanner autos, odometry |
| **Intake** | Over the bumper intake, and also able to grab from feeder station |
| **Shooter** | Flywheel shooter + feeder. Dual barrel. |
| **Hopper** | Controls the hopper. |
| **Climber** | Proto/stub (not yet wired nor fully built) |

- **Controllers:** Xbox controllers, split between driver (0) and operator (1)
- **Vision:** Two Limelights (AprilTags only for now): one parallel with the climber, one parallel with the intake.

---

## Tech Stack

- **WPILib 2026** · **Java 17**
- **CTRE Phoenix 6** — drive and mechanism motors
- **PathPlanner** — autonomous paths and AutoBuilder
- **AdvantageKit** — logging, replay, NT4. Subsystem hardware is isolated behind **IO interfaces** so all inputs are logged and replayable; see [AdvantageKit IO Interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces).
- **Limelights** — vision (PoseEstimate, etc.)
- **SysId** — characterization support
- **Networktables** — advantage kit, easier logging (autolog, shorter syntax)

---

## Getting Started

### Prerequisites

- [WPILib VS Code](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) (or compatible IDE) for 2026
- Java 17
- Team number **2990** set in `.wpilib/wpilib_preferences.json` (already configured)

### Build & Deploy (ya know what to do)

```bash
# Build
./gradlew build

# Deploy to robot (when connected)
./gradlew deploy

# Run tests
./gradlew test
```

- **Simulation:** Not implemented.
- **PathPlanner paths** live in `src/main/deploy/pathplanner/paths/` and are deployed with the project.

### Key Paths/autos

- Endoq auto (primary): Grab fuel from neutral zone, then score.
- Unilateral Depot Auto: Grab fuel from depot, and score.
- Unilateral Outpost Auto: Grab fuel from outpost, and score.

---

## CAN IDs / Wiring Reference

Motor and device IDs are defined in `Constants.MotorIDs`. Use this table as a quick reference for the pit or wiring:

| ID | Constant       | Subsystem | Device / purpose |
| -- | -------------- | --------- | ---------------- |
| 8  | `s_shooterR`   | Shooter   | Shooter right    |
| 9  | `i_follower`   | Intake    | Follower         |
| 11 | `s_feederR`    | Shooter   | Feeder right     |
| 12 | `h_lowerFeed`  | Hopper    | Lower feed       |
| 13 | `i_rollers`    | Intake    | Rollers          |
| 14 | `h_upperFeed`  | Hopper    | Upper feed       |
| 15 | `s_shooterL`   | Shooter   | Shooter left     |

Drive module and Pigeon 2 IDs come from PathPlanner / SysId config (see `TunerConstants` / `vendordeps`).

---

## Troubleshooting

<!-- Add items as you hit issues at the shop or at events. -->

- **Deploy fails / robot not found:** _Check USB/ethernet, driver station connected to same network, team number 2990._
- **Limelight no pose / wrong pose:** _Restart robot, deploy/build code, check pipeline selection and configuration._
- **Shooter RPM unstable or wrong:** _Run a couple more tests, check regression configuration._
- **Auto doesn’t run or path wrong:** _PathPlanner path name matches code, path deployed in `deploy/pathplanner/paths/`, correct path connected._
- **Other:** _Check AdvantageKit logs, NT4 for live values. (call your friendly neighborhood programmer!)_

---

## Competition / Event Checklist

- **Branch:** Use `event/<event-name>` (e.g. `event/bordie`) for competition. Branch from `main` or your current updated branch.

---

## Project Structure

```text
src/main/java/frc/robot/
├── Main.java, Robot.java, RobotContainer.java
├── Constants.java          # IDs, PIDs, poses, game constants
├── ModularSubsystem.java   # Base for subsystems with device management
├── LimelightHelpers.java   # Limelight utilities
├── commands/               # e.g. DriveCommands
├── subsystems/
│   ├── drive/              # Swerve (Drive, Module, GyroIO/ModuleIO + real/sim/no-op impls)
│   ├── intake/             # ProtoIntake (IntakeIO + IntakeIOTalonFX, IntakeIOSim)
│   ├── shooter/            # ProtoShooter (ShooterIO + ShooterIOTalonFX, ShooterIOSim)
│   ├── hopper/             # HopperSubsystem (HopperIO + HopperIOTalonFX, HopperIOSim)
│   └── climber/            # ProtoClimber (ClimberIO stub for future hardware)
├── util/                   # e.g. PhoenixUtil, LocalADStarAK
└── generated/              # e.g. TunerConstants (SysId/characterization)
```

---

## Contributing

- Use the github project plan to mark out changes and issues.
- Make a new branch with yourname/change for general areas of changes. (ex. hotwire-programmer/swerve-setup)
- Never directly commit to main.
- Make all commit messages concise (max 3 words)
- Use pull requests whenever merging into main, and make sure to request review from a mentor or lead programmer.
- Periodically go through branches and merge into main.

---

## License & Credits

- **Team:** [Hotwire Robotics (2990)](https://hotwirerobotics.com)
- **Game:** [FIRST Rebuilt (2026)](https://www.firstinspires.org/robotics/frc)
- **WPILib / FIRST** — [WPILib License](WPILib-License.md)
- **AdvantageKit** — [AdvantageKit License](AdvantageKit-License.md)

---
