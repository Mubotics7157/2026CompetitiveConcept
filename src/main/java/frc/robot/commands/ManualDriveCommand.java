package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Driving;
import frc.robot.subsystems.Swerve;
import frc.util.DriveInputSmoother;
import frc.util.ManualDriveInput;
import frc.util.Stopwatch;
import frc.util.swerve.SwerveSetpoint;

/**
 * Teleop manual drive command for the swerve drivetrain.
 *
 * Handles field-centric driving with manual rotation input and
 * heading-hold behavior after a short delay once rotation input
 * returns to zero.
 *
 * <p>Desired chassis speeds are run through {@link Swerve#generateSetpoint} every loop before
 * being commanded, so the modules only ever get asked to do what they can physically follow -
 * this is what keeps aggressive stick input from breaking traction, rather than reacting to a
 * skid after it's already happened (see {@code SkidDetector}, which this feeds back into).
 */
public class ManualDriveCommand extends Command {
    private enum State {
        IDLING,
        DRIVING_WITH_MANUAL_ROTATION,
        DRIVING_WITH_LOCKED_HEADING
    }

    private static final Time kHeadingLockDelay = Seconds.of(0.25); // time to wait before locking heading
    private static final Time kLoopPeriod = Seconds.of(0.02); // matches the default TimedRobot loop

    private final Swerve swerve;
    private final DriveInputSmoother inputSmoother;
    private final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    private final SwerveRequest.ApplyRobotSpeeds applyRobotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    private final PIDController headingController = new PIDController(5, 0, 0);

    private State currentState = State.IDLING;
    private Optional<Rotation2d> lockedHeading = Optional.empty();
    private Stopwatch headingLockStopwatch = new Stopwatch();
    private ManualDriveInput previousInput = new ManualDriveInput();

    public ManualDriveCommand(
        Swerve swerve,
        DoubleSupplier forwardInput,
        DoubleSupplier leftInput,
        DoubleSupplier rotationInput
    ) {
        this.swerve = swerve;
        this.inputSmoother = new DriveInputSmoother(forwardInput, leftInput, rotationInput);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(swerve);
    }

    public void seedFieldCentric() {
        initialize();
        swerve.seedFieldCentric();
    }

    public void setLockedHeading(Rotation2d heading) {
        lockedHeading = Optional.of(heading);
        currentState = State.DRIVING_WITH_LOCKED_HEADING;
    }

    private void setLockedHeadingToCurrent() {
        final Rotation2d headingInBlueAlliancePerspective = swerve.getState().Pose.getRotation();
        final Rotation2d headingInOperatorPerspective = headingInBlueAlliancePerspective.rotateBy(swerve.getOperatorForwardDirection());
        setLockedHeading(headingInOperatorPerspective);
    }

    private void lockHeadingIfRotationStopped(ManualDriveInput input) {
        if (input.hasRotation()) {
            headingLockStopwatch.reset();
            lockedHeading = Optional.empty();
        } else {
            headingLockStopwatch.startIfNotRunning();
            if (headingLockStopwatch.elapsedTime().gt(kHeadingLockDelay)) {
                setLockedHeadingToCurrent();
            }
        }
    }

    /**
     * Converts a velocity given in operator-perspective (driver "forward"/"left", independent of
     * alliance) into true field-relative, then into robot-relative chassis speeds - the frame the
     * setpoint generator and module kinematics need.
     */
    private ChassisSpeeds toRobotRelative(double forwardMPS, double leftMPS, double omegaRadPerSec) {
        final Translation2d trueFieldVelocity =
            new Translation2d(forwardMPS, leftMPS).rotateBy(swerve.getOperatorForwardDirection().unaryMinus());
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            trueFieldVelocity.getX(),
            trueFieldVelocity.getY(),
            omegaRadPerSec,
            swerve.getState().Pose.getRotation()
        );
    }

    private void driveTowards(ChassisSpeeds desiredRobotRelative) {
        final SwerveSetpoint setpoint = swerve.generateSetpoint(desiredRobotRelative, kLoopPeriod.in(Seconds));
        swerve.setControl(applyRobotSpeedsRequest.withSpeeds(setpoint.robotRelativeSpeeds()));
    }

    @Override
    public void initialize() {
        currentState = State.IDLING;
        lockedHeading = Optional.empty();
        headingLockStopwatch.reset();
        previousInput = new ManualDriveInput();
    }

    @Override
    public void execute() {
        final ManualDriveInput input = inputSmoother.getSmoothedInput();
        if (input.hasRotation()) {
            currentState = State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (input.hasTranslation()) {
            currentState = lockedHeading.isPresent() ? State.DRIVING_WITH_LOCKED_HEADING : State.DRIVING_WITH_MANUAL_ROTATION;
        } else if (previousInput.hasRotation() || previousInput.hasTranslation()) {
            currentState = State.IDLING;
        }
        previousInput = input;

        switch (currentState) {
            case IDLING:
                swerve.setControl(idleRequest);
                swerve.resetSetpoint();
                break;
            case DRIVING_WITH_MANUAL_ROTATION: {
                lockHeadingIfRotationStopped(input);
                final ChassisSpeeds desired = toRobotRelative(
                    Driving.kMaxSpeed.times(input.forward).in(MetersPerSecond),
                    Driving.kMaxSpeed.times(input.left).in(MetersPerSecond),
                    Driving.kMaxRotationalRate.times(input.rotation).in(RadiansPerSecond)
                );
                driveTowards(desired);
                break;
            }
            case DRIVING_WITH_LOCKED_HEADING: {
                final double trueTargetRad =
                    lockedHeading.get().rotateBy(swerve.getOperatorForwardDirection().unaryMinus()).getRadians();
                final double omegaRadPerSec = headingController.calculate(
                    swerve.getState().Pose.getRotation().getRadians(), trueTargetRad
                );
                final ChassisSpeeds desired = toRobotRelative(
                    Driving.kMaxSpeed.times(input.forward).in(MetersPerSecond),
                    Driving.kMaxSpeed.times(input.left).in(MetersPerSecond),
                    omegaRadPerSec
                );
                driveTowards(desired);
                break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        // Default drive command: runs until interrupted
        return false;
    }
}