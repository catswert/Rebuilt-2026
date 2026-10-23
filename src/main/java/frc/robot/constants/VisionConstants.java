package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
    private VisionConstants() {}

    public static final String shootCam = "Arducam_OV9281_USB_Camera";

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Transform3d kRobotToCam = 
                new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, 0, Math.PI));

    /** Estimates further than this off the floor are bad solves. */
    public static final double kMaxPoseZMeters = 0.5;
    /** Single-tag solves flip between two poses above this ambiguity. */
    public static final double kMaxSingleTagAmbiguity = 0.2;
    public static final double kMaxSingleTagDistanceMeters = 4.0;

    /** Position std devs at zero distance; they grow with the square of the distance to the tags. */
    public static final double kSingleTagXyStdDevMeters = 1.0;
    public static final double kMultiTagXyStdDevMeters = 0.3;
    /** Huge so vision never moves the heading; the Pigeon owns it. */
    public static final double kHeadingStdDevRad = 9999.0;
}
