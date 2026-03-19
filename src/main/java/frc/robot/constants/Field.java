package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class Field {

  public static final FieldType type = FieldType.WELDED;

  // AprilTags
  public static final AprilTagFieldLayout tags =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  public static Pose3d tag(int id) {
    return tags.getTagPose(id).get();
  }

  public static final int tagCount = tags.getTags().size();

  public static final Distance tagWidth = Meters.of(Units.inchesToMeters(6.5));

  // Field dimensions
  public static final Distance length = Meters.of(tags.getFieldLength());
  public static final Distance width = Meters.of(tags.getFieldWidth());

  // Game piece
  public static final Distance fuelDiameter = Meters.of(Units.inchesToMeters(5.91));

  /*
   * Vertical Field Lines (X positions)
   */
  public static class XLines {

    public static final Distance center = Meters.of(length.in(Meters) / 2.0);

    public static final Distance starting = Meters.of(tag(26).getX());

    public static final Distance allianceZone = starting;

    public static final Distance hubCenter = Meters.of(tag(26).getX() + Hub.width.in(Meters) / 2.0);

    public static final Distance neutralZoneNear =
        Meters.of(center.in(Meters) - Units.inchesToMeters(120));

    public static final Distance neutralZoneFar =
        Meters.of(center.in(Meters) + Units.inchesToMeters(120));

    public static final Distance oppHubCenter =
        Meters.of(tag(4).getX() + Hub.width.in(Meters) / 2.0);

    public static final Distance oppAllianceZone = Meters.of(tag(10).getX());
  }

  /*
   * Horizontal Field Lines (Y positions)
   */
  public static class YLines {

    public static final Distance center = Meters.of(width.in(Meters) / 2.0);

    public static final Distance rightBumpStart = Meters.of(Hub.nearRightCorner.getY());

    public static final Distance rightBumpEnd =
        Meters.of(rightBumpStart.in(Meters) - RightBump.width.in(Meters));

    public static final Distance rightBumpMiddle =
        Meters.of((rightBumpStart.in(Meters) + rightBumpEnd.in(Meters)) / 2.0);

    public static final Distance rightTrenchOpenStart =
        Meters.of(rightBumpEnd.in(Meters) - Units.inchesToMeters(12));

    public static final Distance rightTrenchOpenEnd = Meters.of(0);

    public static final Distance leftBumpEnd = Meters.of(Hub.nearLeftCorner.getY());

    public static final Distance leftBumpStart =
        Meters.of(leftBumpEnd.in(Meters) + LeftBump.width.in(Meters));

    public static final Distance leftBumpMiddle =
        Meters.of((leftBumpStart.in(Meters) + leftBumpEnd.in(Meters)) / 2.0);

    public static final Distance leftTrenchOpenEnd =
        Meters.of(leftBumpStart.in(Meters) + Units.inchesToMeters(12));

    public static final Distance leftTrenchOpenStart = width;
  }

  /*
   * Hub
   */
  public static class Hub {

    public static final Distance width = Meters.of(Units.inchesToMeters(47));

    public static final Distance height = Meters.of(Units.inchesToMeters(72));

    public static final Distance innerWidth = Meters.of(Units.inchesToMeters(41.7));

    public static final Distance innerHeight = Meters.of(Units.inchesToMeters(56.5));

    public static final Translation3d topCenterPoint =
        new Translation3d(
            tag(26).getX() + width.in(Meters) / 2, Field.width.in(Meters) / 2, height.in(Meters));

    public static final Translation3d innerCenterPoint =
        new Translation3d(
            tag(26).getX() + width.in(Meters) / 2,
            Field.width.in(Meters) / 2,
            innerHeight.in(Meters));

    public static final Translation2d nearLeftCorner =
        new Translation2d(
            topCenterPoint.getX() - width.in(Meters) / 2,
            Field.width.in(Meters) / 2 + width.in(Meters) / 2);

    public static final Translation2d nearRightCorner =
        new Translation2d(
            topCenterPoint.getX() - width.in(Meters) / 2,
            Field.width.in(Meters) / 2 - width.in(Meters) / 2);

    public static final Translation2d farLeftCorner =
        new Translation2d(
            topCenterPoint.getX() + width.in(Meters) / 2,
            Field.width.in(Meters) / 2 + width.in(Meters) / 2);

    public static final Translation2d farRightCorner =
        new Translation2d(
            topCenterPoint.getX() + width.in(Meters) / 2,
            Field.width.in(Meters) / 2 - width.in(Meters) / 2);

    public static final Pose2d nearFace = tag(26).toPose2d();

    public static final Pose2d farFace = tag(20).toPose2d();

    public static final Pose2d rightFace = tag(18).toPose2d();

    public static final Pose2d leftFace = tag(21).toPose2d();
  }

  /*
   * Left Bump
   */
  public static class LeftBump {

    public static final Distance width = Meters.of(Units.inchesToMeters(73));

    public static final Distance height = Meters.of(Units.inchesToMeters(6.513));

    public static final Distance depth = Meters.of(Units.inchesToMeters(44.4));

    public static final Translation2d nearLeftCorner =
        new Translation2d(
            XLines.hubCenter.in(Meters) - width.in(Meters) / 2, Units.inchesToMeters(255));

    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;

    public static final Translation2d farLeftCorner =
        new Translation2d(
            XLines.hubCenter.in(Meters) + width.in(Meters) / 2, Units.inchesToMeters(255));

    public static final Translation2d farRightCorner = Hub.farLeftCorner;
  }

  /*
   * Right Bump
   */
  public static class RightBump {

    public static final Distance width = Meters.of(Units.inchesToMeters(73));

    public static final Distance height = Meters.of(Units.inchesToMeters(6.513));

    public static final Distance depth = Meters.of(Units.inchesToMeters(44.4));

    public static final Translation2d nearLeftCorner =
        new Translation2d(
            XLines.hubCenter.in(Meters) + width.in(Meters) / 2, Units.inchesToMeters(255));

    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;

    public static final Translation2d farLeftCorner =
        new Translation2d(
            XLines.hubCenter.in(Meters) - width.in(Meters) / 2, Units.inchesToMeters(255));

    public static final Translation2d farRightCorner = Hub.farLeftCorner;
  }

  /*
   * Tower
   */
  public static class Tower {

    public static final Distance width = Meters.of(Units.inchesToMeters(49.25));

    public static final Distance depth = Meters.of(Units.inchesToMeters(45));

    public static final Distance height = Meters.of(Units.inchesToMeters(78.25));

    public static final Distance innerOpeningWidth = Meters.of(Units.inchesToMeters(32.25));

    public static final Distance frontFaceX = Meters.of(Units.inchesToMeters(43.51));

    public static final Translation2d centerPoint =
        new Translation2d(frontFaceX.in(Meters), tag(31).getY());
  }

  public static class Depot {

    public static final Distance width = Meters.of(Units.inchesToMeters(42));

    public static final Distance depth = Meters.of(Units.inchesToMeters(27));

    public static final Distance height = Meters.of(Units.inchesToMeters(1.125));

    public static final Distance distanceFromCenterY = Meters.of(Units.inchesToMeters(75.93));

    public static final Translation3d depotCenter =
        new Translation3d(
            depth.in(Meters),
            (Field.width.in(Meters) / 2) + distanceFromCenterY.in(Meters),
            height.in(Meters));
  }

  public static class Outpost {

    public static final Distance width = Meters.of(Units.inchesToMeters(31.8));

    public static final Distance openingDistanceFromFloor = Meters.of(Units.inchesToMeters(28.1));

    public static final Distance height = Meters.of(Units.inchesToMeters(7));

    public static final Translation2d centerPoint = new Translation2d(0, tag(29).getY());
  }

  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    private final String jsonFolder;

    FieldType(String s) {
      this.jsonFolder = s;
    }
  }
}
