package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.constants.OperatorConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;
import frc.robot.util.HeadingSnap;
import frc.robot.util.PointToHeading;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;



@Logged
public class RobotContainer {
    private final CommandXboxController driver = new CommandXboxController(RobotConstants.DRIVER_PORT);
    private final CommandXboxController operator = new CommandXboxController(RobotConstants.OPERATOR_PORT);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer();
    public final Turret turret = new Turret();
    public final Hood hood = new Hood();
    public final Shooter shooter = new Shooter();
    public final Vision vision = new Vision(drivetrain::addVisionMeasurement);
    private final SendableChooser<Command> autoChooser;

    public final StateMachine stateMachine = new StateMachine(
            drivetrain, intake, indexer, turret, hood, shooter, vision,
            operator::getRightX,
            () -> -operator.getLeftY());

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(RobotConstants.DRIVE_MAX_SPEED_MPS * RobotConstants.DRIVE_TRANSLATION_DEADBAND_FRACTION)
            .withRotationalDeadband(RobotConstants.DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC * RobotConstants.DRIVE_ROTATION_DEADBAND_FRACTION)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    /** No rotational deadband: the heading controller's small holding corrections must reach the modules. */
    private final SwerveRequest.FieldCentric pointDrive = new SwerveRequest.FieldCentric()
            .withDeadband(RobotConstants.DRIVE_MAX_SPEED_MPS * RobotConstants.DRIVE_TRANSLATION_DEADBAND_FRACTION)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final PointToHeading pointToHeading = new PointToHeading();
    @Logged(importance = Logged.Importance.CRITICAL) private boolean pointToHeadingEnabled = false;
    private final HeadingSnap headingSnap = new HeadingSnap();
    private static final double PRECISION_RAMP_PER_SEC =
            (1.0 - RobotConstants.PRECISION_SPEED_SCALE) / RobotConstants.PRECISION_TRANSITION_SECONDS;
    private final SlewRateLimiter speedScaleLimiter =
            new SlewRateLimiter(PRECISION_RAMP_PER_SEC, -PRECISION_RAMP_PER_SEC, 1.0);
    @Logged(importance = Logged.Importance.CRITICAL) private double speedScale = 1.0;
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Telemetry telemetry = new Telemetry(RobotConstants.DRIVE_MAX_SPEED_MPS);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        registerNamedCommands();
        configureDriverBindings();
        configureOperatorBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureDriverBindings() {
        drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> {
            boolean precision = driver.getRightTriggerAxis() > RobotConstants.PRECISION_TRIGGER_THRESHOLD;
            speedScale = speedScaleLimiter.calculate(precision ? RobotConstants.PRECISION_SPEED_SCALE : 1.0);
            double velocityX = -driver.getLeftY() * RobotConstants.DRIVE_MAX_SPEED_MPS * speedScale;
            double velocityY = -driver.getLeftX() * RobotConstants.DRIVE_MAX_SPEED_MPS * speedScale;
            Rotation2d snapHeading = headingSnap.updateDpad(driver.getHID().getPOV(), RobotConstants.LOOP_PERIOD_SECONDS);
            if (snapHeading != null) {
                headingSnap.request(snapHeading, drivetrain.getState().Speeds.omegaRadiansPerSecond);
            }
            // Moving the rotation stick always takes over from a snap.
            if (Math.abs(driver.getRightX()) >= RobotConstants.DRIVE_ROTATION_DEADBAND_FRACTION) {
                headingSnap.cancel();
            }
            Rotation2d operatorHeading = drivetrain.getState().Pose.getRotation()
                    .minus(drivetrain.getOperatorForwardDirection());
            if (pointToHeadingEnabled) {
                // Point-to-heading keeps its full turn rate in precision mode; only driving slows.
                return pointDrive.withVelocityX(velocityX).withVelocityY(velocityY)
                        .withRotationalRate(pointToHeading.rotationalRate(driver.getRightX(), driver.getRightY(), operatorHeading));
            }
            if (headingSnap.isActive()) {
                // Snap holds its heading until the stick moves, a new direction is pressed, or the field is reseeded.
                return pointDrive.withVelocityX(velocityX).withVelocityY(velocityY)
                        .withRotationalRate(headingSnap.rotationalRate(operatorHeading));
            }
            return drive.withVelocityX(velocityX).withVelocityY(velocityY)
                    .withRotationalRate(-driver.getRightX() * RobotConstants.DRIVE_MAX_ANGULAR_RATE_RAD_PER_SEC * speedScale);
        }));

        RobotModeTriggers.disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
        // A heading held from before a disable or a reseed would snap the robot back to it.
        RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> {
            pointToHeading.reset();
            headingSnap.cancel();
        }).ignoringDisable(true));

        // Both back paddles (mapped to X and B in Flydigi Space Station) X-lock the wheels.
        driver.x().and(driver.b()).whileTrue(drivetrain.applyRequest(() -> brake));
        // LM + RM (mapped to LB and RB) reseed the field-relative heading.
        driver.leftBumper().and(driver.rightBumper()).onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            pointToHeading.reset();
            headingSnap.cancel();
        }));
        // Test feature: right stick points the intake instead of setting a turn rate.
        driver.rightStick().onTrue(Commands.runOnce(() -> {
            pointToHeadingEnabled = !pointToHeadingEnabled;
            pointToHeading.reset();
        }));

        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    private void configureOperatorBindings() {
        //operator.x.onTrue for allat runOnce and shi for toggleManualTurret
        operator.x().toggleOnTrue(Commands.runOnce(stateMachine::toggleManualTurret));
        operator.a().onTrue(Commands.runOnce(stateMachine::toggleIntake));
        operator.y().onTrue(Commands.runOnce(stateMachine::togglePassing));

        operator.rightTrigger().whileTrue(Commands.startEnd(
                () -> stateMachine.setShoot(true),
                () -> stateMachine.setShoot(false)));

        operator.rightStick().whileTrue(Commands.startEnd(
                () -> stateMachine.setClear(true),
                () -> stateMachine.setClear(false)));

        operator.povUp().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_UP_ROT)));
        operator.povRight().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_RIGHT_ROT)));
        operator.povLeft().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_LEFT_ROT)));
        operator.povDown().onTrue(Commands.runOnce(() -> stateMachine.setHoodSetpoint(OperatorConstants.HOOD_PRESET_DOWN_ROT)));

        operator.back().onTrue(Commands.runOnce(() -> {
            turret.zero();
            stateMachine.setTurretSetpoint(0.0);
        }));
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("ToggleIntake", Commands.runOnce(stateMachine::toggleIntake));
        NamedCommands.registerCommand("TogglePassing", Commands.runOnce(stateMachine::togglePassing));
        NamedCommands.registerCommand("Shoot", Commands.startEnd(
            () -> stateMachine.setShoot(true),
            () -> stateMachine.setShoot(false)));

        NamedCommands.registerCommand("Clear", Commands.startEnd(
            () -> stateMachine.setClear(true),
            () -> stateMachine.setClear(false)));
    }

}
