package frc.robot.subsystems.gamepieces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simulates loose game pieces for desktop robot simulation.
 *
 * <p>Includes:
 *
 * <ul>
 *   <li>2D collisions between game pieces
 *   <li>field wall collisions
 *   <li>intake pickup when robot is intaking
 *   <li>single held piece and simple shooter projectile launch
 *   <li>Pose3d logging for AdvantageScope 3D visualization/replay
 * </ul>
 */
public class GamePieceSim extends SubsystemBase {
  private static final double FIELD_LENGTH_METERS = 16.54;
  private static final double FIELD_WIDTH_METERS = 8.21;
  // REBUILT (2026) FUEL is 5.91 in (15.0 cm) diameter => 2.955 in radius.
  private static final double PIECE_RADIUS_METERS = 0.075057;
  // User-provided robot footprint: 33 in x 33 in.
  private static final double ROBOT_HALF_LENGTH_METERS = 0.4191;
  private static final double ROBOT_HALF_WIDTH_METERS = 0.4191;
  private static final double ROBOT_COLLISION_HEIGHT_METERS = 0.35;

  // REBUILT element dimensions (manual section 5.x), modeled as simple 2D blockers with height.
  private static final double HUB_SIZE_METERS = 1.1938; // 47 in
  private static final double HUB_HEIGHT_METERS = 1.20; // Approximate rigid body height
  private static final double BUMP_WIDTH_METERS = 1.8542; // 73.0 in
  private static final double BUMP_DEPTH_METERS = 1.1280; // 44.4 in
  private static final double BUMP_HEIGHT_METERS = 0.1654; // 6.513 in
  private static final double TOWER_WIDTH_METERS = 1.2510; // 49.25 in
  private static final double TOWER_DEPTH_METERS = 1.1430; // 45.0 in
  private static final double TOWER_HEIGHT_METERS = 1.9880; // 78.25 in
  private static final double TRENCH_WIDTH_METERS = 1.6680; // 65.65 in
  private static final double TRENCH_DEPTH_METERS = 1.1940; // 47.0 in
  private static final double TRENCH_HEIGHT_METERS = 1.0220; // 40.25 in
  private static final double DEPOT_WIDTH_METERS = 1.0700; // 42.0 in
  private static final double DEPOT_DEPTH_METERS = 0.6860; // 27.0 in
  private static final double DEPOT_HEIGHT_METERS = 0.0286; // 1.125 in
  private static final double OUTPOST_WIDTH_METERS = 0.9080; // 35.8 in corral width
  private static final double OUTPOST_DEPTH_METERS = 0.9550; // 37.6 in corral depth
  private static final double OUTPOST_HEIGHT_METERS = 0.2060; // 8.13 in
  private static final double PIECE_RESTITUTION = 0.45;
  private static final double WALL_RESTITUTION = 0.6;
  private static final double STRUCTURE_RESTITUTION = 0.35;
  private static final double ROBOT_RESTITUTION = 0.25;
  private static final double GROUND_DAMPING_PER_TICK = 0.98;
  private static final double AIR_DAMPING_PER_TICK = 0.995;
  private static final double GRAVITY_METERS_PER_SEC2 = 9.81;
  private static final double PICKUP_RADIUS_METERS = 0.24;
  private static final double SHOOT_SPEED_METERS_PER_SEC = 8.5;
  private static final double SHOOT_UP_SPEED_METERS_PER_SEC = 2.75;

  private final Supplier<Pose2d> robotPoseSupplier;
  private final BooleanSupplier intakeActiveSupplier;
  private final BooleanSupplier shooterFiringSupplier;
  private final BooleanSupplier simEnabledSupplier;

  private final List<PieceState> pieces = new ArrayList<>();
  private final List<FieldCollider> fieldColliders = new ArrayList<>();
  private PieceState heldPiece = null;
  private boolean previousShooterState = false;
  private double lastTimestampSeconds = 0.0;

  /**
   * Creates the game piece simulation model.
   *
   * @param robotPoseSupplier current robot pose supplier
   * @param intakeActiveSupplier true when intake should pick up pieces
   * @param shooterFiringSupplier true when shooter should launch held piece
   * @param simEnabledSupplier true when simulation updates should run
   */
  public GamePieceSim(
      Supplier<Pose2d> robotPoseSupplier,
      BooleanSupplier intakeActiveSupplier,
      BooleanSupplier shooterFiringSupplier,
      BooleanSupplier simEnabledSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    this.intakeActiveSupplier = intakeActiveSupplier;
    this.shooterFiringSupplier = shooterFiringSupplier;
    this.simEnabledSupplier = simEnabledSupplier;
    resetFieldPieces();
    buildFieldColliders();
  }

  /** Resets all game pieces to default spawn points. */
  public final void resetFieldPieces() {
    pieces.clear();
    heldPiece = null;
    previousShooterState = false;

    // A simple line of spawn points near midfield for testing interactions.
    spawnPiece(new Translation2d(3.0, 2.0));
    spawnPiece(new Translation2d(4.0, 2.0));
    spawnPiece(new Translation2d(5.0, 2.0));
    spawnPiece(new Translation2d(3.0, 3.2));
    spawnPiece(new Translation2d(4.2, 3.4));
    spawnPiece(new Translation2d(5.1, 3.1));
  }

  /**
   * Spawns one ground piece at the provided XY location.
   *
   * @param locationMeters location in field coordinates
   */
  public void spawnPiece(Translation2d locationMeters) {
    pieces.add(new PieceState(locationMeters));
  }

  @Override
  public void periodic() {
    if (!simEnabledSupplier.getAsBoolean()) {
      logOutputs();
      return;
    }

    final double nowSeconds = Timer.getFPGATimestamp();
    if (lastTimestampSeconds == 0.0) {
      lastTimestampSeconds = nowSeconds;
    }
    final double dtSeconds = clamp(nowSeconds - lastTimestampSeconds, 0.005, 0.03);
    lastTimestampSeconds = nowSeconds;

    final Pose2d robotPose = robotPoseSupplier.get();

    // Keep held piece attached to robot.
    updateHeldPose(robotPose);

    // Intake captures a nearby piece if robot has none held.
    if (heldPiece == null && intakeActiveSupplier.getAsBoolean()) {
      attemptPickup(robotPose);
    }

    // Fire held piece on rising edge of shooter "firing" state.
    final boolean shooterNow = shooterFiringSupplier.getAsBoolean();
    if (heldPiece != null && shooterNow && !previousShooterState) {
      launchHeldPiece(robotPose);
    }
    previousShooterState = shooterNow;

    integratePieceMotion(dtSeconds);
    resolvePieceCollisions();
    resolveRobotCollision(robotPose);
    resolveFieldStructureCollisions();
    clampPiecesToField();
    resolveScoring();
    logOutputs();
  }

  /** Attempts to pick up the nearest piece in intake range. */
  private void attemptPickup(Pose2d robotPose) {
    final Translation2d intakePoint =
        robotPose.transformBy(new Transform2d(0.45, 0.0, Rotation2d.kZero)).getTranslation();
    PieceState nearest = null;
    double nearestDistance = Double.POSITIVE_INFINITY;

    for (PieceState piece : pieces) {
      if (piece.scored || piece.held || piece.zMeters > 0.02) {
        continue;
      }
      final double distance = piece.xyMeters.getDistance(intakePoint);
      if (distance <= PICKUP_RADIUS_METERS && distance < nearestDistance) {
        nearestDistance = distance;
        nearest = piece;
      }
    }

    if (nearest != null) {
      nearest.held = true;
      nearest.zMeters = 0.20;
      nearest.vzMetersPerSec = 0.0;
      nearest.velocityMetersPerSec = new Translation2d();
      heldPiece = nearest;
    }
  }

  /** Launches the held piece with a basic projectile velocity model. */
  private void launchHeldPiece(Pose2d robotPose) {
    if (heldPiece == null) {
      return;
    }
    final Translation2d muzzle =
        robotPose.transformBy(new Transform2d(0.35, 0.0, Rotation2d.kZero)).getTranslation();
    final double headingRadians = robotPose.getRotation().getRadians();
    final Translation2d launchVelocity =
        new Translation2d(
            SHOOT_SPEED_METERS_PER_SEC * Math.cos(headingRadians),
            SHOOT_SPEED_METERS_PER_SEC * Math.sin(headingRadians));

    heldPiece.held = false;
    heldPiece.xyMeters = muzzle;
    heldPiece.zMeters = 0.55;
    heldPiece.velocityMetersPerSec = launchVelocity;
    heldPiece.vzMetersPerSec = SHOOT_UP_SPEED_METERS_PER_SEC;
    heldPiece = null;
  }

  /** Updates the held piece pose to remain attached to the robot. */
  private void updateHeldPose(Pose2d robotPose) {
    if (heldPiece == null) {
      return;
    }
    heldPiece.xyMeters =
        robotPose.transformBy(new Transform2d(0.20, 0.0, Rotation2d.kZero)).getTranslation();
    heldPiece.zMeters = 0.20;
    heldPiece.velocityMetersPerSec = new Translation2d();
    heldPiece.vzMetersPerSec = 0.0;
  }

  /** Integrates each piece using simple ground and projectile dynamics. */
  private void integratePieceMotion(double dtSeconds) {
    for (PieceState piece : pieces) {
      if (piece.scored || piece.held) {
        continue;
      }

      piece.xyMeters = piece.xyMeters.plus(piece.velocityMetersPerSec.times(dtSeconds));

      if (piece.zMeters > 0.0 || Math.abs(piece.vzMetersPerSec) > 1e-3) {
        piece.vzMetersPerSec -= GRAVITY_METERS_PER_SEC2 * dtSeconds;
        piece.zMeters += piece.vzMetersPerSec * dtSeconds;
        piece.velocityMetersPerSec = piece.velocityMetersPerSec.times(AIR_DAMPING_PER_TICK);
        if (piece.zMeters <= 0.0) {
          piece.zMeters = 0.0;
          if (Math.abs(piece.vzMetersPerSec) > 1.0) {
            piece.vzMetersPerSec = -piece.vzMetersPerSec * 0.35;
          } else {
            piece.vzMetersPerSec = 0.0;
          }
          piece.velocityMetersPerSec = piece.velocityMetersPerSec.times(0.8);
        }
      } else {
        piece.velocityMetersPerSec = piece.velocityMetersPerSec.times(GROUND_DAMPING_PER_TICK);
        if (piece.velocityMetersPerSec.getNorm() < 0.03) {
          piece.velocityMetersPerSec = new Translation2d();
        }
      }
    }
  }

  /** Resolves pairwise 2D collisions for non-held, non-scored ground pieces. */
  private void resolvePieceCollisions() {
    final double minDistance = PIECE_RADIUS_METERS * 2.0;
    for (int i = 0; i < pieces.size(); i++) {
      final PieceState a = pieces.get(i);
      if (a.scored || a.held || a.zMeters > 0.02) {
        continue;
      }
      for (int j = i + 1; j < pieces.size(); j++) {
        final PieceState b = pieces.get(j);
        if (b.scored || b.held || b.zMeters > 0.02) {
          continue;
        }

        final Translation2d delta = a.xyMeters.minus(b.xyMeters);
        final double distance = delta.getNorm();
        if (distance < 1e-6 || distance >= minDistance) {
          continue;
        }

        final Translation2d normal = delta.div(distance);
        final Translation2d relativeVelocity = a.velocityMetersPerSec.minus(b.velocityMetersPerSec);
        final double normalRelativeSpeed =
            relativeVelocity.getX() * normal.getX() + relativeVelocity.getY() * normal.getY();

        if (normalRelativeSpeed < 0.0) {
          final double impulse = -(1.0 + PIECE_RESTITUTION) * normalRelativeSpeed / 2.0;
          a.velocityMetersPerSec = a.velocityMetersPerSec.plus(normal.times(impulse));
          b.velocityMetersPerSec = b.velocityMetersPerSec.minus(normal.times(impulse));
        }

        final double overlap = minDistance - distance;
        final Translation2d correction = normal.times(overlap * 0.5);
        a.xyMeters = a.xyMeters.plus(correction);
        b.xyMeters = b.xyMeters.minus(correction);
      }
    }
  }

  /** Resolves collisions against the robot's oriented 33x33in rectangular footprint. */
  private void resolveRobotCollision(Pose2d robotPose) {
    final Rotation2d robotRotation = robotPose.getRotation();
    final double cos = Math.cos(robotRotation.getRadians());
    final double sin = Math.sin(robotRotation.getRadians());
    final Translation2d robotCenter = robotPose.getTranslation();

    for (PieceState piece : pieces) {
      if (piece.scored || piece.held || piece.zMeters > ROBOT_COLLISION_HEIGHT_METERS) {
        continue;
      }

      final Translation2d rel = piece.xyMeters.minus(robotCenter);
      final double localX = rel.getX() * cos + rel.getY() * sin;
      final double localY = -rel.getX() * sin + rel.getY() * cos;

      final double clampedX = clamp(localX, -ROBOT_HALF_LENGTH_METERS, ROBOT_HALF_LENGTH_METERS);
      final double clampedY = clamp(localY, -ROBOT_HALF_WIDTH_METERS, ROBOT_HALF_WIDTH_METERS);
      final double dx = localX - clampedX;
      final double dy = localY - clampedY;
      final double distance = Math.hypot(dx, dy);

      if (distance >= PIECE_RADIUS_METERS) {
        continue;
      }

      // Outward normal in robot-local space. If center is inside, pick shortest axis exit.
      double normalLocalX;
      double normalLocalY;
      if (distance > 1e-6) {
        normalLocalX = dx / distance;
        normalLocalY = dy / distance;
      } else {
        final double xPen = ROBOT_HALF_LENGTH_METERS - Math.abs(localX);
        final double yPen = ROBOT_HALF_WIDTH_METERS - Math.abs(localY);
        if (xPen < yPen) {
          normalLocalX = Math.signum(localX == 0.0 ? 1.0 : localX);
          normalLocalY = 0.0;
        } else {
          normalLocalX = 0.0;
          normalLocalY = Math.signum(localY == 0.0 ? 1.0 : localY);
        }
      }

      final Translation2d normalWorld =
          new Translation2d(
              normalLocalX * cos - normalLocalY * sin, normalLocalX * sin + normalLocalY * cos);

      final double overlap = PIECE_RADIUS_METERS - distance;
      piece.xyMeters = piece.xyMeters.plus(normalWorld.times(overlap + 1e-4));

      final double vn =
          piece.velocityMetersPerSec.getX() * normalWorld.getX()
              + piece.velocityMetersPerSec.getY() * normalWorld.getY();
      if (vn < 0.0) {
        piece.velocityMetersPerSec =
            piece.velocityMetersPerSec.minus(normalWorld.times((1.0 + ROBOT_RESTITUTION) * vn));
      }
    }
  }

  /** Resolves collisions against major field structures (HUB/BUMP/TOWER/TRENCH/DEPOT/OUTPOST). */
  private void resolveFieldStructureCollisions() {
    for (PieceState piece : pieces) {
      if (piece.scored || piece.held) {
        continue;
      }
      for (FieldCollider collider : fieldColliders) {
        if (piece.zMeters > collider.heightMeters) {
          continue; // Airborne over this structure, no collision
        }

        final double minX = collider.center.getX() - collider.halfX;
        final double maxX = collider.center.getX() + collider.halfX;
        final double minY = collider.center.getY() - collider.halfY;
        final double maxY = collider.center.getY() + collider.halfY;

        final double closestX = clamp(piece.xyMeters.getX(), minX, maxX);
        final double closestY = clamp(piece.xyMeters.getY(), minY, maxY);
        final double dx = piece.xyMeters.getX() - closestX;
        final double dy = piece.xyMeters.getY() - closestY;
        final double distance = Math.hypot(dx, dy);

        if (distance >= PIECE_RADIUS_METERS) {
          continue;
        }

        Translation2d normal;
        if (distance > 1e-6) {
          normal = new Translation2d(dx / distance, dy / distance);
        } else {
          // Piece center landed inside collider bounds; push towards shallowest boundary.
          final double left = Math.abs(piece.xyMeters.getX() - minX);
          final double right = Math.abs(maxX - piece.xyMeters.getX());
          final double down = Math.abs(piece.xyMeters.getY() - minY);
          final double up = Math.abs(maxY - piece.xyMeters.getY());
          final double smallest = Math.min(Math.min(left, right), Math.min(down, up));
          if (smallest == left) {
            normal = new Translation2d(-1.0, 0.0);
          } else if (smallest == right) {
            normal = new Translation2d(1.0, 0.0);
          } else if (smallest == down) {
            normal = new Translation2d(0.0, -1.0);
          } else {
            normal = new Translation2d(0.0, 1.0);
          }
        }

        final double overlap = PIECE_RADIUS_METERS - distance;
        piece.xyMeters = piece.xyMeters.plus(normal.times(overlap + 1e-4));
        final double vn =
            piece.velocityMetersPerSec.getX() * normal.getX()
                + piece.velocityMetersPerSec.getY() * normal.getY();
        if (vn < 0.0) {
          piece.velocityMetersPerSec =
              piece.velocityMetersPerSec.minus(normal.times((1.0 + STRUCTURE_RESTITUTION) * vn));
        }
      }
    }
  }

  /** Keeps pieces inside field bounds and applies bounce on wall impact. */
  private void clampPiecesToField() {
    for (PieceState piece : pieces) {
      if (piece.scored || piece.held) {
        continue;
      }
      double x = piece.xyMeters.getX();
      double y = piece.xyMeters.getY();
      double vx = piece.velocityMetersPerSec.getX();
      double vy = piece.velocityMetersPerSec.getY();

      if (x < PIECE_RADIUS_METERS) {
        x = PIECE_RADIUS_METERS;
        vx = Math.abs(vx) * WALL_RESTITUTION;
      } else if (x > FIELD_LENGTH_METERS - PIECE_RADIUS_METERS) {
        x = FIELD_LENGTH_METERS - PIECE_RADIUS_METERS;
        vx = -Math.abs(vx) * WALL_RESTITUTION;
      }

      if (y < PIECE_RADIUS_METERS) {
        y = PIECE_RADIUS_METERS;
        vy = Math.abs(vy) * WALL_RESTITUTION;
      } else if (y > FIELD_WIDTH_METERS - PIECE_RADIUS_METERS) {
        y = FIELD_WIDTH_METERS - PIECE_RADIUS_METERS;
        vy = -Math.abs(vy) * WALL_RESTITUTION;
      }

      piece.xyMeters = new Translation2d(x, y);
      piece.velocityMetersPerSec = new Translation2d(vx, vy);
    }
  }

  /** Marks projectile pieces as scored when they pass through the hub/tower area. */
  private void resolveScoring() {
    final Translation2d hubLocation = Constants.Poses.hub.getTranslation();
    for (PieceState piece : pieces) {
      if (piece.scored || piece.held) {
        continue;
      }
      if (piece.zMeters > 0.9 && piece.xyMeters.getDistance(hubLocation) < 0.55) {
        piece.scored = true;
        piece.velocityMetersPerSec = new Translation2d();
        piece.vzMetersPerSec = 0.0;
        piece.zMeters = 0.0;
      }
    }
  }

  /** Builds a set of simplified static colliders for major field objects on both alliances. */
  private void buildFieldColliders() {
    fieldColliders.clear();
    addAllianceFieldColliders(false); // blue side
    addAllianceFieldColliders(true); // red side
  }

  /**
   * Adds one alliance's structures using known/derived locations.
   *
   * <p>Positions are approximations based on 2026 manual dimensions and current field constants.
   */
  private void addAllianceFieldColliders(boolean redAlliance) {
    final Pose2d hubPose =
        redAlliance
            ? Constants.flipAlliance(new Pose2d(4.625594, 3.965, Rotation2d.k180deg))
            : new Pose2d(4.625594, 3.965, Rotation2d.k180deg);
    final Pose2d towerPose =
        redAlliance
            ? Constants.flipAlliance(new Pose2d(1.5653, 4.146, Rotation2d.k180deg))
            : new Pose2d(1.5653, 4.146, Rotation2d.k180deg);

    // HUB body
    fieldColliders.add(
        new FieldCollider(
            hubPose.getTranslation(),
            HUB_SIZE_METERS * 0.5,
            HUB_SIZE_METERS * 0.5,
            HUB_HEIGHT_METERS));

    // Two BUMPs around each HUB (modeled as axis-aligned rectangles offset in Y).
    final double bumpYOffset = HUB_SIZE_METERS * 0.5 + BUMP_WIDTH_METERS * 0.5;
    fieldColliders.add(
        new FieldCollider(
            hubPose.getTranslation().plus(new Translation2d(0.0, bumpYOffset)),
            BUMP_DEPTH_METERS * 0.5,
            BUMP_WIDTH_METERS * 0.5,
            BUMP_HEIGHT_METERS));
    fieldColliders.add(
        new FieldCollider(
            hubPose.getTranslation().plus(new Translation2d(0.0, -bumpYOffset)),
            BUMP_DEPTH_METERS * 0.5,
            BUMP_WIDTH_METERS * 0.5,
            BUMP_HEIGHT_METERS));

    // TOWER base/body
    fieldColliders.add(
        new FieldCollider(
            towerPose.getTranslation(),
            TOWER_DEPTH_METERS * 0.5,
            TOWER_WIDTH_METERS * 0.5,
            TOWER_HEIGHT_METERS));

    // DEPOT along alliance wall (approximate center: near tower side).
    final double depotCenterX =
        redAlliance ? FIELD_LENGTH_METERS - DEPOT_DEPTH_METERS * 0.5 : DEPOT_DEPTH_METERS * 0.5;
    final double depotCenterY = towerPose.getY() + 1.0;
    fieldColliders.add(
        new FieldCollider(
            new Translation2d(depotCenterX, depotCenterY),
            DEPOT_DEPTH_METERS * 0.5,
            DEPOT_WIDTH_METERS * 0.5,
            DEPOT_HEIGHT_METERS));

    // OUTPOST corral at both field side edges for this alliance end.
    final double outpostCenterX =
        redAlliance ? FIELD_LENGTH_METERS - OUTPOST_DEPTH_METERS * 0.5 : OUTPOST_DEPTH_METERS * 0.5;
    fieldColliders.add(
        new FieldCollider(
            new Translation2d(outpostCenterX, OUTPOST_WIDTH_METERS * 0.5),
            OUTPOST_DEPTH_METERS * 0.5,
            OUTPOST_WIDTH_METERS * 0.5,
            OUTPOST_HEIGHT_METERS));
    fieldColliders.add(
        new FieldCollider(
            new Translation2d(outpostCenterX, FIELD_WIDTH_METERS - OUTPOST_WIDTH_METERS * 0.5),
            OUTPOST_DEPTH_METERS * 0.5,
            OUTPOST_WIDTH_METERS * 0.5,
            OUTPOST_HEIGHT_METERS));

    // TRENCH arms near guardrails around the neutral-zone depth near HUB.
    final double trenchCenterX = redAlliance ? FIELD_LENGTH_METERS - 4.50 : 4.50;
    fieldColliders.add(
        new FieldCollider(
            new Translation2d(trenchCenterX, TRENCH_DEPTH_METERS * 0.5),
            TRENCH_WIDTH_METERS * 0.5,
            TRENCH_DEPTH_METERS * 0.5,
            TRENCH_HEIGHT_METERS));
    fieldColliders.add(
        new FieldCollider(
            new Translation2d(trenchCenterX, FIELD_WIDTH_METERS - TRENCH_DEPTH_METERS * 0.5),
            TRENCH_WIDTH_METERS * 0.5,
            TRENCH_DEPTH_METERS * 0.5,
            TRENCH_HEIGHT_METERS));
  }

  /** Emits 3D piece poses and counts for AdvantageScope visual replay/debugging. */
  private void logOutputs() {
    final List<Pose3d> ground = new ArrayList<>();
    final List<Pose3d> airborne = new ArrayList<>();
    Pose3d held = new Pose3d();
    boolean hasHeld = false;
    int scoredCount = 0;

    for (PieceState piece : pieces) {
      if (piece.scored) {
        scoredCount++;
        continue;
      }
      final Pose3d pose = toPose3d(piece);
      if (piece.held) {
        held = pose;
        hasHeld = true;
      } else if (piece.zMeters > 0.04) {
        airborne.add(pose);
      } else {
        ground.add(pose);
      }
    }

    Logger.recordOutput("GamePieceSim/GroundPieces", ground.toArray(new Pose3d[0]));
    Logger.recordOutput("GamePieceSim/AirbornePieces", airborne.toArray(new Pose3d[0]));
    Logger.recordOutput("GamePieceSim/HasHeldPiece", hasHeld);
    if (hasHeld) {
      Logger.recordOutput("GamePieceSim/HeldPiece", held);
    }
    Logger.recordOutput("GamePieceSim/ScoredCount", scoredCount);
    Logger.recordOutput("GamePieceSim/TotalCount", pieces.size());
  }

  /** Converts a piece state to a field Pose3d for AdvantageScope rendering. */
  private static Pose3d toPose3d(PieceState piece) {
    return new Pose3d(
        new Translation3d(piece.xyMeters.getX(), piece.xyMeters.getY(), piece.zMeters),
        new Rotation3d());
  }

  /** Simple scalar clamp helper. */
  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  /** Internal mutable game piece physics state. */
  private static class PieceState {
    private Translation2d xyMeters;
    private Translation2d velocityMetersPerSec = new Translation2d();
    private double zMeters = 0.0;
    private double vzMetersPerSec = 0.0;
    private boolean held = false;
    private boolean scored = false;

    /** Creates a piece at a ground location. */
    private PieceState(Translation2d xyMeters) {
      this.xyMeters = xyMeters;
    }
  }

  /** Axis-aligned 2D rectangle collider with an effective collision height. */
  private static class FieldCollider {
    private final Translation2d center;
    private final double halfX;
    private final double halfY;
    private final double heightMeters;

    /** Creates a rectangular field collider. */
    private FieldCollider(Translation2d center, double halfX, double halfY, double heightMeters) {
      this.center = center;
      this.halfX = halfX;
      this.halfY = halfY;
      this.heightMeters = heightMeters;
    }
  }
}
