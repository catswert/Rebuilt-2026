package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotConstants;

/**
 * Test driving mode: the rotation stick points the intake in a field direction, and how far the
 * stick is pushed caps how fast the robot turns there. Inside the deadband the robot holds the
 * heading it had when the stick came back to center. Headings are operator relative, so stick
 * up is straight away from the driver.
 */
public final class PointToHeading {
    private Rotation2d heldHeading = null;

    /** Forget the held heading so the next centered stick locks wherever the robot is then. */
    public void reset() {
        heldHeading = null;
    }

    /** Rotational rate in rad/s from the raw stick axes (right is +x, down is +y, as the controller reports). */
    public double rotationalRate(double stickX, double stickY, Rotation2d currentHeading) {
        double magnitude = stickMagnitude(stickX, stickY);
        if (magnitude == 0.0) {
            if (heldHeading == null) {
                heldHeading = currentHeading;
            }
            return rate(errorRad(currentHeading, heldHeading), 1.0);
        }
        heldHeading = null;
        return rate(errorRad(currentHeading, stickHeading(stickX, stickY)), magnitude);
    }

    /** Stick up faces away from the driver and stick left faces left, counterclockwise positive. */
    static Rotation2d stickHeading(double stickX, double stickY) {
        return new Rotation2d(-stickY, -stickX);
    }

    /** Radial deadband, rescaled so speed starts from zero at the deadband edge and reaches 1 at full deflection. */
    static double stickMagnitude(double stickX, double stickY) {
        double raw = Math.min(Math.hypot(stickX, stickY), 1.0);
        return MathUtil.applyDeadband(raw, RobotConstants.DRIVE_ROTATION_DEADBAND_FRACTION);
    }

    /** Shortest turn from current to target, counterclockwise positive. A target exactly behind turns counterclockwise. */
    static double errorRad(Rotation2d current, Rotation2d target) {
        double error = MathUtil.angleModulus(target.getRadians() - current.getRadians());
        if (Math.abs(Math.abs(error) - Math.PI) < RobotConstants.POINT_HEADING_TIE_TOLERANCE_RAD) {
            return Math.PI;
        }
        return error;
    }

    /** Proportional turn toward the error, capped at the stick's share of the max turn rate. */
    static double rate(double errorRad, double magnitude) {
        double cap = magnitude * RobotConstants.DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC;
        return MathUtil.clamp(RobotConstants.POINT_HEADING_KP * errorRad, -cap, cap);
    }
}
