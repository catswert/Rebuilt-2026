package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.RobotConstants;

class PointToHeadingTest {
    private static final double MAX_RATE = RobotConstants.DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC;

    @Test
    void stickDirectionIsOperatorRelative() {
        assertEquals(0.0, PointToHeading.stickHeading(0.0, -1.0).getDegrees(), 1e-9);
        assertEquals(90.0, PointToHeading.stickHeading(-1.0, 0.0).getDegrees(), 1e-9);
        assertEquals(-90.0, PointToHeading.stickHeading(1.0, 0.0).getDegrees(), 1e-9);
        assertEquals(180.0, Math.abs(PointToHeading.stickHeading(0.0, 1.0).getDegrees()), 1e-9);
    }

    @Test
    void magnitudeIsRadiallyDeadbandedAndCapped() {
        assertEquals(0.0, PointToHeading.stickMagnitude(0.05, 0.05));
        assertEquals(1.0, PointToHeading.stickMagnitude(1.0, 1.0), 1e-9);
        double half = PointToHeading.stickMagnitude(0.0, 0.5);
        assertTrue(half > 0.0 && half < 0.5);
    }

    @Test
    void errorTakesTheShortWayAcrossTheWrap() {
        assertEquals(Math.toRadians(20.0),
                PointToHeading.errorRad(Rotation2d.fromDegrees(170.0), Rotation2d.fromDegrees(-170.0)), 1e-9);
        assertEquals(Math.toRadians(-20.0),
                PointToHeading.errorRad(Rotation2d.fromDegrees(-170.0), Rotation2d.fromDegrees(170.0)), 1e-9);
    }

    @Test
    void targetExactlyBehindTurnsCounterclockwise() {
        assertEquals(Math.PI, PointToHeading.errorRad(Rotation2d.kZero, Rotation2d.k180deg), 1e-9);
        assertEquals(Math.PI, PointToHeading.errorRad(Rotation2d.k180deg, Rotation2d.kZero), 1e-9);
        assertEquals(Math.PI, PointToHeading.errorRad(Rotation2d.kCCW_Pi_2, Rotation2d.kCW_Pi_2), 1e-9);
        assertEquals(Math.PI, PointToHeading.errorRad(Rotation2d.kCW_Pi_2, Rotation2d.kCCW_Pi_2), 1e-9);
    }

    @Test
    void stickDeflectionCapsTurnRate() {
        assertEquals(MAX_RATE, PointToHeading.rate(Math.PI, 1.0), 1e-9);
        assertEquals(0.5 * MAX_RATE, PointToHeading.rate(Math.PI, 0.5), 1e-9);
        assertEquals(-0.5 * MAX_RATE, PointToHeading.rate(-Math.PI, 0.5), 1e-9);
        assertEquals(RobotConstants.POINT_HEADING_KP * 0.01, PointToHeading.rate(0.01, 1.0), 1e-9);
    }

    @Test
    void centeredStickHoldsTheHeadingItReleasedAt() {
        PointToHeading pointToHeading = new PointToHeading();
        Rotation2d released = Rotation2d.fromDegrees(30.0);
        assertEquals(0.0, pointToHeading.rotationalRate(0.0, 0.0, released), 1e-9);
        // Bumped clockwise, so it turns counterclockwise back toward 30 degrees.
        assertTrue(pointToHeading.rotationalRate(0.0, 0.0, Rotation2d.fromDegrees(20.0)) > 0.0);
    }

    @Test
    void movingTheStickReleasesTheHold() {
        PointToHeading pointToHeading = new PointToHeading();
        pointToHeading.rotationalRate(0.0, 0.0, Rotation2d.kZero);
        // Stick hard right points the intake right, a clockwise turn.
        assertTrue(pointToHeading.rotationalRate(1.0, 0.0, Rotation2d.kZero) < 0.0);
        // Once there, back to center holds the new heading, not the old one.
        assertEquals(0.0, pointToHeading.rotationalRate(0.0, 0.0, Rotation2d.kCW_Pi_2), 1e-9);
    }

    @Test
    void quickFlickStillTurnsAllTheWay() {
        PointToHeading pointToHeading = new PointToHeading();
        // A flick right, released before the robot has turned much.
        pointToHeading.rotationalRate(1.0, 0.0, Rotation2d.kZero);
        Rotation2d partWay = Rotation2d.fromDegrees(-20.0);
        // Centered, it keeps turning clockwise toward -90 at the full rate cap.
        assertEquals(-MAX_RATE, pointToHeading.rotationalRate(0.0, 0.0, partWay), 1e-9);
        assertEquals(0.0, pointToHeading.rotationalRate(0.0, 0.0, Rotation2d.kCW_Pi_2), 1e-9);
    }

    @Test
    void resetLocksWhereverTheRobotIs() {
        PointToHeading pointToHeading = new PointToHeading();
        pointToHeading.rotationalRate(1.0, 0.0, Rotation2d.kZero);
        pointToHeading.reset();
        assertEquals(0.0, pointToHeading.rotationalRate(0.0, 0.0, Rotation2d.fromDegrees(-20.0)), 1e-9);
    }
}
