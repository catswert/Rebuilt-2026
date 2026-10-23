package frc.robot.subsystems;

import java.util.Comparator;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TurretConstants;
import frc.robot.constants.VisionConstants;

@Logged
public class Vision extends SubsystemBase {
    @NotLogged private final PhotonCamera shootCam;
    @NotLogged private final PhotonPoseEstimator photonEstimator;
    @NotLogged private final EstimateConsumer estConsumer;

    public Vision(EstimateConsumer estConsumer) {
        shootCam = new PhotonCamera(VisionConstants.shootCam);
        photonEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout, VisionConstants.kRobotToCam);
        this.estConsumer = estConsumer;
    }

    public final void update() {
        for (var result : shootCam.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> multiTagEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (multiTagEst.isPresent()) {
                accept(multiTagEst.get(), true);
            } else {
                photonEstimator.estimateLowestAmbiguityPose(result).ifPresent(est -> accept(est, false));
            }
        }
    }

    private void accept(EstimatedRobotPose est, boolean multiTag) {
        stdDevsFor(est, multiTag).ifPresent(
                stdDevs -> estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, stdDevs));
    }

    /**
     * How much to trust an estimate, or empty to throw it away. Heading is effectively never
     * trusted: the Pigeon is far better than a tag, and field-centric driving follows the pose heading.
     */
    private static Optional<Matrix<N3, N1>> stdDevsFor(EstimatedRobotPose est, boolean multiTag) {
        Pose3d pose = est.estimatedPose;
        boolean onField = pose.getX() >= 0.0 && pose.getX() <= VisionConstants.kTagLayout.getFieldLength()
                && pose.getY() >= 0.0 && pose.getY() <= VisionConstants.kTagLayout.getFieldWidth()
                && Math.abs(pose.getZ()) <= VisionConstants.kMaxPoseZMeters;
        if (!onField || est.targetsUsed.isEmpty()) {
            return Optional.empty();
        }

        double baseXyStdDev;
        double distanceMeters;
        if (multiTag) {
            baseXyStdDev = VisionConstants.kMultiTagXyStdDevMeters;
            distanceMeters = est.targetsUsed.stream()
                    .mapToDouble(Vision::cameraToTagMeters).average().orElse(0.0);
        } else {
            PhotonTrackedTarget best = est.targetsUsed.stream()
                    .filter(target -> target.getPoseAmbiguity() >= 0.0)
                    .min(Comparator.comparingDouble(PhotonTrackedTarget::getPoseAmbiguity))
                    .orElse(null);
            if (best == null || best.getPoseAmbiguity() > VisionConstants.kMaxSingleTagAmbiguity) {
                return Optional.empty();
            }
            distanceMeters = cameraToTagMeters(best);
            if (distanceMeters > VisionConstants.kMaxSingleTagDistanceMeters) {
                return Optional.empty();
            }
            baseXyStdDev = VisionConstants.kSingleTagXyStdDevMeters;
        }

        double xyStdDev = baseXyStdDev * (1.0 + distanceMeters * distanceMeters / 30.0);
        return Optional.of(VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.kHeadingStdDevRad));
    }

    private static double cameraToTagMeters(PhotonTrackedTarget target) {
        return target.getBestCameraToTarget().getTranslation().getNorm();
    }

    public final double updateTurretSetpoint(Pose2d drivetrainPose, Translation2d hub){
        double hubBearingRot = bearingRot(drivetrainPose, hub);
        double turretSetpointRot = turretRotForBearing(hubBearingRot);
        return turretSetpointRot;

    }

    public final double updateTurretPassingSetpoint(Pose2d drivetrainPose, Rotation2d fieldHeading){
        double passingBearingRot = fieldHeading.minus(drivetrainPose.getRotation()).getRotations();
        return turretRotForBearing(passingBearingRot);
    }

        /** Angle from the intake to the target, counterclockwise-positive, in [-0.5, 0.5]. */
    public static double bearingRot(Pose2d robotPose, Translation2d target) {
        Rotation2d fieldAngleToTarget = target.minus(robotPose.getTranslation()).getAngle();
        return fieldAngleToTarget.minus(robotPose.getRotation()).getRotations();
    }

    /**
     * Turret position that points along a bearing. The turret counts clockwise from its boot
     * heading while bearings count counterclockwise, so the position is boot heading minus bearing.
     * Turret.setSetpoint wraps the result into the travel limits.
     */
    public static double turretRotForBearing(double bearingRot) {
        return TurretConstants.BOOT_HEADING_ROT - bearingRot;
    }

    /** Pose at the robot center facing the field direction the turret points: robot heading plus boot heading minus turret position. */
    public static Pose2d turretFieldPose(Pose2d robotPose, double turretRot) {
        Rotation2d turretRelativeToRobot = Rotation2d.fromRotations(TurretConstants.BOOT_HEADING_ROT - turretRot);
        return new Pose2d(robotPose.getTranslation(), robotPose.getRotation().plus(turretRelativeToRobot));
    }

    @FunctionalInterface
    public interface EstimateConsumer {
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);
    }
}
