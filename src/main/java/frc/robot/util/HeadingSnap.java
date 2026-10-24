package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotConstants;

/**
 * D-pad heading snap: each of the 8 D-pad directions turns the robot to that operator relative
 * heading (up is straight away from the driver, counterclockwise positive) and holds it. A direction
 * must be held {@link RobotConstants#SNAP_HOLD_SECONDS} before it counts, and after a diagonal fires,
 * the cardinal still held when one finger lifts is ignored until the D-pad is fully released.
 */
public final class HeadingSnap {
    private static final int NONE = -1;

    private int candidate = NONE;
    private double heldSeconds = 0.0;
    private boolean fired = false;
    private int guard = NONE;

    private Rotation2d target = null;
    private double lastRate = 0.0;

    /**
     * Feed the raw POV angle (degrees clockwise from up, -1 when released) once per loop.
     * Returns the heading to snap to on the loop a direction counts, otherwise null.
     */
    public Rotation2d updateDpad(int povDegrees, double dtSeconds) {
        int octant = octant(povDegrees);
        if (octant == NONE) {
            candidate = NONE;
            heldSeconds = 0.0;
            fired = false;
            guard = NONE;
            return null;
        }
        if (octant != candidate) {
            candidate = octant;
            heldSeconds = 0.0;
            fired = false;
            return null;
        }
        heldSeconds += dtSeconds;
        // Tolerance so summed loop periods that land a hair under the hold time still count.
        if (fired || heldSeconds < RobotConstants.SNAP_HOLD_SECONDS - 1e-9) {
            return null;
        }
        fired = true;
        if (guard != NONE && octant % 2 == 0 && adjacent(octant, guard)) {
            return null;
        }
        if (octant % 2 == 1) {
            guard = octant;
        }
        return heading(octant);
    }

    /** Start (or retarget) a snap. A new snap picks up from the measured spin so the turn never jolts. */
    public void request(Rotation2d heading, double measuredRateRadPerSec) {
        if (target == null) {
            lastRate = measuredRateRadPerSec;
        }
        target = heading;
    }

    /** Drop the held heading, e.g. when the rotation stick takes over or the field is reseeded. */
    public void cancel() {
        target = null;
    }

    public boolean isActive() {
        return target != null;
    }

    /** Turn rate in rad/s toward the snap heading. Call once per loop while active. */
    public double rotationalRate(Rotation2d currentHeading) {
        double error = PointToHeading.errorRad(currentHeading, target);
        double magnitude = Math.abs(error);
        double rate = Math.signum(error) * Math.min(RobotConstants.SNAP_MAX_RATE_RAD_PER_SEC,
                Math.min(RobotConstants.SNAP_KP * magnitude,
                        Math.sqrt(2.0 * RobotConstants.SNAP_DECEL_RAD_PER_SEC_SQ * magnitude)));
        double step = RobotConstants.SNAP_ACCEL_RAD_PER_SEC_SQ * RobotConstants.LOOP_PERIOD_SECONDS;
        lastRate = MathUtil.clamp(rate, lastRate - step, lastRate + step);
        return lastRate;
    }

    /** D-pad octant counterclockwise from up (0 = up, 2 = left, 6 = right), or NONE when released. */
    static int octant(int povDegrees) {
        if (povDegrees < 0) {
            return NONE;
        }
        return Math.floorMod(-Math.round(povDegrees / 45.0f), 8);
    }

    static Rotation2d heading(int octant) {
        return Rotation2d.fromRadians(MathUtil.angleModulus(octant * Math.PI / 4.0));
    }

    private static boolean adjacent(int a, int b) {
        return Math.floorMod(a - b, 8) == 1 || Math.floorMod(b - a, 8) == 1;
    }
}
