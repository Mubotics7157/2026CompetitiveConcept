package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Traction;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.util.swerve.ModuleLimits;
import frc.util.swerve.SkidDetector;
import frc.util.swerve.SwerveSetpoint;
import frc.util.swerve.SwerveSetpointGenerator;

public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds pathFieldSpeedsRequest = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);

    /* Module locations, in FrontLeft/FrontRight/BackLeft/BackRight order to match the order
     * modules are constructed in below (and therefore the order getState().ModuleStates reports
     * them in). Shared by the setpoint generator and skid detector so both agree with the
     * drivetrain about which module is which. */
    private final Translation2d[] moduleLocations = new Translation2d[] {
        new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
        new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
        new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
        new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY),
    };
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleLocations);

    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        kinematics, moduleLocations, Traction.kMassKg, Traction.kWheelCoefficientOfFriction
    );
    private final SkidDetector skidDetector = new SkidDetector(kinematics, Traction.kSkidToleranceMPS);
    private SwerveSetpoint previousSetpoint;

    public Swerve() {
        super(
            TunerConstants.DrivetrainConstants, 
            0,
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.1, 0.1, 0.1),
            TunerConstants.FrontLeft, 
            TunerConstants.FrontRight, 
            TunerConstants.BackLeft, 
            TunerConstants.BackRight
        );
        previousSetpoint = SwerveSetpoint.zero(getState().ModuleStates);
    }

    /**
     * Steps the drivetrain's remembered setpoint one loop toward the desired robot-relative
     * chassis speeds, respecting drive acceleration, steering slew rate, and traction limits.
     * While a skid is detected, acceleration is derated to help the wheels recover grip before
     * asking for more.
     *
     * <p>Callers should feed back the returned setpoint's speeds as the actual command (e.g. via
     * {@code SwerveRequest.ApplyRobotSpeeds}), and call this again next loop - the generator needs
     * to see its own previous output, not the measured robot state, to remain consistent.
     *
     * @param desiredRobotRelativeSpeeds Desired robot-relative chassis speeds.
     * @param dt Loop period, in seconds.
     * @return The next feasible setpoint.
     */
    public SwerveSetpoint generateSetpoint(ChassisSpeeds desiredRobotRelativeSpeeds, double dt) {
        double accelScale = skidDetector.isSkidding() ? Traction.kSkidAccelerationScale : 1.0;
        ModuleLimits limits = new ModuleLimits(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            Traction.kMaxLinearAccelerationMPSSq * accelScale,
            Traction.kMaxSteeringVelocity.in(RadiansPerSecond)
        );
        previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, desiredRobotRelativeSpeeds, limits, dt);
        return previousSetpoint;
    }

    /**
     * Resets the remembered setpoint to zero speed at the current module angles. Call this
     * whenever the drivetrain isn't being actively driven through {@link #generateSetpoint} (e.g.
     * idling or disabled), so the next call starts cleanly instead of ramping from a stale speed.
     */
    public void resetSetpoint() {
        previousSetpoint = SwerveSetpoint.zero(getState().ModuleStates);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {});
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {
        return new AutoFactory(
            () -> getState().Pose,
            this::resetPose,
            this::followPath,
            true,
            this,
            trajLogger
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += pathXController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += pathYController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += pathThetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        setControl(
            pathFieldSpeedsRequest.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }

        skidDetector.update(getState().ModuleStates);
        SmartDashboard.putBoolean("Swerve/Skidding", skidDetector.isSkidding());
        SmartDashboard.putNumber("Swerve/SkidRatio", skidDetector.getSkidRatio());

        if (DriverStation.isDisabled()) {
            // Don't let a stale high-speed setpoint sit around while disabled - start clean
            // whenever driving resumes.
            resetSetpoint();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}