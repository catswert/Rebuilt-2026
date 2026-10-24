package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotConstants;

class HeadingSnapTest {
    private static final double DT = RobotConstants.LOOP_PERIOD_SECONDS;
    private static final int RELEASED = -1;

    /** Holds the POV for the given number of loops and returns the first heading it fired, or null. */
    private static Rotation2d hold(HeadingSnap snap, int pov, int loops) {
        Rotation2d fired = null;
        for (int i = 0; i < loops; i++) {
            Rotation2d heading = snap.updateDpad(pov, DT);
            if (fired == null) {
                fired = heading;
            }
        }
        return fired;
    }

    @Test
    void dpadHeadingsAreOperatorRelativeCounterclockwise() {
        assertEquals(0.0, HeadingSnap.heading(HeadingSnap.octant(0)).getDegrees(), 1e-9);
        assertEquals(-45.0, HeadingSnap.heading(HeadingSnap.octant(45)).getDegrees(), 1e-9);
        assertEquals(-90.0, HeadingSnap.heading(HeadingSnap.octant(90)).getDegrees(), 1e-9);
        assertEquals(-135.0, HeadingSnap.heading(HeadingSnap.octant(135)).getDegrees(), 1e-9);
        assertEquals(180.0, Math.abs(HeadingSnap.heading(HeadingSnap.octant(180)).getDegrees()), 1e-9);
        assertEquals(135.0, HeadingSnap.heading(HeadingSnap.octant(225)).getDegrees(), 1e-9);
        assertEquals(90.0, HeadingSnap.heading(HeadingSnap.octant(270)).getDegrees(), 1e-9);
        assertEquals(45.0, HeadingSnap.heading(HeadingSnap.octant(315)).getDegrees(), 1e-9);
    }

    @Test
    void directionMustBeHeldBeforeItCounts() {
        HeadingSnap snap = new HeadingSnap();
        int loopsToFire = (int) Math.round(RobotConstants.SNAP_HOLD_SECONDS / DT) + 1;
        assertNull(hold(snap, 90, loopsToFire - 1));
        assertEquals(-90.0, snap.updateDpad(90, DT).getDegrees(), 1e-9);
        // Fires once per press, not every loop it stays held.
        assertNull(hold(snap, 90, 20));
    }

    @Test
    void tapShorterThanHoldTimeIsIgnored() {
        HeadingSnap snap = new HeadingSnap();
        assertNull(hold(snap, 0, 3));
        assertNull(hold(snap, RELEASED, 1));
        assertNull(hold(snap, 0, 3));
    }

    @Test
    void rollingThroughADirectionRestartsTheHold() {
        HeadingSnap snap = new HeadingSnap();
        assertNull(hold(snap, 0, 4));
        assertNull(hold(snap, 45, 4));
        assertEquals(-90.0, hold(snap, 90, 10).getDegrees(), 1e-9);
    }

    @Test
    void leftoverCardinalAfterADiagonalIsIgnoredUntilRelease() {
        HeadingSnap snap = new HeadingSnap();
        assertEquals(-45.0, hold(snap, 45, 10).getDegrees(), 1e-9);
        // One finger lifts and up is still held: ignored.
        assertNull(hold(snap, 0, 20));
        // A non-adjacent direction still works while the pad is held.
        assertEquals(90.0, hold(snap, 270, 10).getDegrees(), 1e-9);
        // After a full release the same cardinal counts again.
        hold(snap, RELEASED, 1);
        assertEquals(0.0, hold(snap, 0, 10).getDegrees(), 1e-9);
    }

    @Test
    void cancelDropsTheHeldHeading() {
        HeadingSnap snap = new HeadingSnap();
        assertFalse(snap.isActive());
        snap.request(Rotation2d.kCCW_Pi_2, 0.0);
        assertTrue(snap.isActive());
        snap.cancel();
        assertFalse(snap.isActive());
    }

    @Test
    void turnCommandRampsFromTheMeasuredSpin() {
        HeadingSnap snap = new HeadingSnap();
        double step = RobotConstants.SNAP_ACCEL_RAD_PER_SEC_SQ * DT;
        snap.request(Rotation2d.kCCW_Pi_2, 0.0);
        assertEquals(step, snap.rotationalRate(Rotation2d.kZero), 1e-9);
        assertEquals(2.0 * step, snap.rotationalRate(Rotation2d.kZero), 1e-9);

        // Already spinning clockwise: the command eases out of it instead of jumping.
        HeadingSnap spinning = new HeadingSnap();
        spinning.request(Rotation2d.kCCW_Pi_2, -2.0);
        assertEquals(-2.0 + step, spinning.rotationalRate(Rotation2d.kZero), 1e-9);
    }

    @Test
    void retargetingKeepsTheCurrentCommand() {
        HeadingSnap snap = new HeadingSnap();
        snap.request(Rotation2d.kCCW_Pi_2, 0.0);
        double rate = snap.rotationalRate(Rotation2d.kZero);
        snap.request(Rotation2d.kCW_Pi_2, 3.0);
        assertEquals(rate - RobotConstants.SNAP_ACCEL_RAD_PER_SEC_SQ * DT,
                snap.rotationalRate(Rotation2d.kZero), 1e-9);
    }

    @Test
    void ninetyDegreeSnapCruisesAndStopsWithoutOvershoot() {
        HeadingSnap snap = new HeadingSnap();
        Rotation2d target = Rotation2d.kCCW_Pi_2;
        snap.request(target, 0.0);
        double heading = 0.0;
        double peak = 0.0;
        double settledSeconds = Double.NaN;
        for (int i = 0; i < 150; i++) {
            double rate = snap.rotationalRate(Rotation2d.fromRadians(heading));
            peak = Math.max(peak, rate);
            heading += rate * DT;
            assertTrue(heading <= target.getRadians() + 1e-6, "overshot the snap heading");
            if (Double.isNaN(settledSeconds) && target.getRadians() - heading < Math.toRadians(1.0)) {
                settledSeconds = (i + 1) * DT;
            }
        }
        assertEquals(RobotConstants.SNAP_MAX_RATE_RAD_PER_SEC, peak, 1e-9);
        assertTrue(settledSeconds < 1.5, "took " + settledSeconds + " s");
        assertEquals(target.getRadians(), heading, Math.toRadians(0.5));
    }

    @Test
    void snapTakesTheShortWayAround() {
        HeadingSnap snap = new HeadingSnap();
        snap.request(Rotation2d.fromDegrees(-170.0), 0.0);
        assertTrue(snap.rotationalRate(Rotation2d.fromDegrees(170.0)) > 0.0);
    }
}
