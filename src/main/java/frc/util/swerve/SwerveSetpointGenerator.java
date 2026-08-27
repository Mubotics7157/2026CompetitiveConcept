package frc.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Generates kinematically-feasible swerve setpoints, adapted from FRC team 254's algorithm (the
 * same one PathPlannerLib's {@code SwerveSetpointGenerator} is built on). Given the previous
 * commanded setpoint and a newly-desired chassis speed, this steps each module's angle and speed
 * toward the goal only as fast as it is physically able to - clamping steering slew rate, drive
 * acceleration, and how quickly a loaded wheel is allowed to change heading before it needs more
 * lateral grip than the carpet can provide.
 *
 * <p>This is the main lever for "clean and smooth" driving: most wheel skid on a swerve comes
 * from commanding an instantaneous module angle/speed jump (e.g. driver snaps the stick from one
 * corner to the opposite corner) that the physical wheel simply cannot track, so it slips instead
 * of tracking. Interpolating feasibly, loop to loop, prevents that in the first place rather than
 * reacting to it after the fact.
 *
 * <p>Unlike the version vendored via PathPlannerLib in some codebases, this build has no
 * dependency on PathPlannerLib or a motor torque model - it works purely off
 * {@link SwerveDriveKinematics}, module locations, robot mass, and a wheel/carpet friction
 * coefficient, all of which this project already has or can supply as simple constants.
 */
public class SwerveSetpointGenerator {
    private static final double kEpsilon = 1e-6;

    private final SwerveDriveKinematics kinematics;
    private final Translation2d[] moduleLocations;
    private final double massKg;
    private final double wheelCoefficientOfFriction;

    /**
     * @param kinematics Kinematics built from the same module locations as the drivetrain.
     * @param moduleLocations Module locations relative to robot center, in the same order used to
     *     build {@code kinematics} (and the same order the drivetrain reports module states in).
     * @param massKg Robot mass, in kilograms.
     * @param wheelCoefficientOfFriction Coefficient of friction between the wheel and the carpet.
     *     Roughly 1.0-1.5 for treaded wheels; tune against the real robot.
     */
    public SwerveSetpointGenerator(
            SwerveDriveKinematics kinematics,
            Translation2d[] moduleLocations,
            double massKg,
            double wheelCoefficientOfFriction) {
        this.kinematics = kinematics;
        this.moduleLocations = moduleLocations;
        this.massKg = massKg;
        this.wheelCoefficientOfFriction = wheelCoefficientOfFriction;
    }

    /**
     * Generate the next feasible setpoint.
     *
     * @param prevSetpoint The setpoint generated (and commanded) last loop. Pass the actual
     *     returned setpoint back in next time - not the measured robot state - so the generator's
     *     notion of "where the wheels currently are" stays consistent.
     * @param desiredRobotRelative The desired robot-relative chassis speeds, e.g. from driver
     *     input or a trajectory follower. Do not discretize this yourself - it happens internally.
     * @param limits Kinematic limits to respect this loop.
     * @param dt Loop period, in seconds.
     * @return The next feasible setpoint, converging toward {@code desiredRobotRelative} as fast
     *     as the limits allow.
     */
    public SwerveSetpoint generateSetpoint(
            SwerveSetpoint prevSetpoint,
            ChassisSpeeds desiredRobotRelative,
            ModuleLimits limits,
            double dt) {
        final int numModules = moduleLocations.length;

        SwerveModuleState[] desiredModuleStates =
                kinematics.toSwerveModuleStates(desiredRobotRelative);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, limits.maxDriveVelocityMPS());
        desiredRobotRelative = kinematics.toChassisSpeeds(desiredModuleStates);

        // Special case: desired is a complete stop. Module angle is arbitrary, so keep the
        // previous angle rather than snapping to whatever angle a zero-speed state happens to be.
        boolean needToSteer = true;
        if (epsilonEquals(desiredRobotRelative, new ChassisSpeeds())) {
            needToSteer = false;
            for (int m = 0; m < numModules; m++) {
                desiredModuleStates[m].angle = prevSetpoint.moduleStates()[m].angle;
                desiredModuleStates[m].speedMetersPerSecond = 0.0;
            }
        }

        double[] prevVx = new double[numModules];
        double[] prevVy = new double[numModules];
        Rotation2d[] prevHeading = new Rotation2d[numModules];
        double[] desiredVx = new double[numModules];
        double[] desiredVy = new double[numModules];
        Rotation2d[] desiredHeading = new Rotation2d[numModules];
        boolean allModulesShouldFlip = true;

        for (int m = 0; m < numModules; m++) {
            prevVx[m] =
                    prevSetpoint.moduleStates()[m].angle.getCos()
                            * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            prevVy[m] =
                    prevSetpoint.moduleStates()[m].angle.getSin()
                            * prevSetpoint.moduleStates()[m].speedMetersPerSecond;
            prevHeading[m] = prevSetpoint.moduleStates()[m].angle;
            if (prevSetpoint.moduleStates()[m].speedMetersPerSecond < 0.0) {
                prevHeading[m] = prevHeading[m].rotateBy(Rotation2d.k180deg);
            }

            desiredVx[m] = desiredModuleStates[m].angle.getCos() * desiredModuleStates[m].speedMetersPerSecond;
            desiredVy[m] = desiredModuleStates[m].angle.getSin() * desiredModuleStates[m].speedMetersPerSecond;
            desiredHeading[m] = desiredModuleStates[m].angle;
            if (desiredModuleStates[m].speedMetersPerSecond < 0.0) {
                desiredHeading[m] = desiredHeading[m].rotateBy(Rotation2d.k180deg);
            }

            if (allModulesShouldFlip) {
                double requiredRotationRad =
                        Math.abs(prevHeading[m].unaryMinus().rotateBy(desiredHeading[m]).getRadians());
                if (requiredRotationRad < Math.PI / 2.0) {
                    allModulesShouldFlip = false;
                }
            }
        }

        if (allModulesShouldFlip
                && !epsilonEquals(prevSetpoint.robotRelativeSpeeds(), new ChassisSpeeds())
                && !epsilonEquals(desiredRobotRelative, new ChassisSpeeds())) {
            // Every module would need to flip >90 degrees - faster to stop, rotate modules to the
            // complement angle, then accelerate back out, than to try to slew all the way around.
            return generateSetpoint(prevSetpoint, new ChassisSpeeds(), limits, dt);
        }

        final double dx = desiredRobotRelative.vxMetersPerSecond - prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond;
        final double dy = desiredRobotRelative.vyMetersPerSecond - prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond;
        final double dtheta =
                desiredRobotRelative.omegaRadiansPerSecond
                        - prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond;

        // 's' interpolates between the previous state (s=0) and the desired state (s=1). We solve
        // for the largest s every module can achieve without violating a limit, and use the
        // smallest of those across all modules.
        double minS = 1.0;

        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(numModules);
        final double wheelFrictionForce = massKg * 9.81 * wheelCoefficientOfFriction;

        for (int m = 0; m < numModules; m++) {
            if (!needToSteer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[m].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());

            double maxThetaStep = dt * limits.maxSteeringVelocityRadPerSec();

            if (epsilonEquals(prevSetpoint.moduleStates()[m].speedMetersPerSecond, 0.0)) {
                // Module is stopped - it will need to steer straight to the goal angle before it
                // can start driving, so limit purely on rotation-in-place.
                if (epsilonEquals(desiredModuleStates[m].speedMetersPerSecond, 0.0)) {
                    overrideSteering.set(m, Optional.of(prevSetpoint.moduleStates()[m].angle));
                    continue;
                }

                var necessaryRotation =
                        prevSetpoint.moduleStates()[m].angle.unaryMinus().rotateBy(desiredModuleStates[m].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(Rotation2d.kPi);
                }

                final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / maxThetaStep;
                if (numStepsNeeded <= 1.0) {
                    overrideSteering.set(m, Optional.of(desiredModuleStates[m].angle));
                } else {
                    overrideSteering.set(
                            m,
                            Optional.of(
                                    prevSetpoint.moduleStates()[m].angle.rotateBy(
                                            Rotation2d.fromRadians(
                                                    Math.signum(necessaryRotation.getRadians()) * maxThetaStep))));
                    minS = 0.0;
                }
                continue;
            }
            if (minS == 0.0) {
                continue;
            }

            // Centripetal / traction limit: cap how fast a rolling module's heading can change so
            // that turning it doesn't demand more lateral grip than wheel-friction force allows.
            // This is the piece that most directly keeps aggressive stick snaps from breaking
            // traction - without it, the steering-velocity limit above is the only limit, and
            // that alone still lets a fast-moving wheel be steered into a skid.
            double maxHeadingChange =
                    (dt * wheelFrictionForce)
                            / ((massKg / numModules)
                                    * Math.abs(prevSetpoint.moduleStates()[m].speedMetersPerSecond));
            maxThetaStep = Math.min(maxThetaStep, maxHeadingChange);

            double s =
                    findSteeringMaxS(
                            prevVx[m],
                            prevVy[m],
                            prevHeading[m].getRadians(),
                            desiredVx[m],
                            desiredVy[m],
                            desiredHeading[m].getRadians(),
                            maxThetaStep);
            minS = Math.min(minS, s);
        }

        for (int m = 0; m < numModules; m++) {
            if (minS == 0.0) {
                break;
            }

            double maxVelStep = limits.maxDriveAccelerationMPSSq() * dt;

            double vxMinS = minS == 1.0 ? desiredVx[m] : (desiredVx[m] - prevVx[m]) * minS + prevVx[m];
            double vyMinS = minS == 1.0 ? desiredVy[m] : (desiredVy[m] - prevVy[m]) * minS + prevVy[m];

            double s = findDriveMaxS(prevVx[m], prevVy[m], vxMinS, vyMinS, maxVelStep);
            minS = Math.min(minS, s);
        }

        ChassisSpeeds retSpeeds =
                new ChassisSpeeds(
                        prevSetpoint.robotRelativeSpeeds().vxMetersPerSecond + minS * dx,
                        prevSetpoint.robotRelativeSpeeds().vyMetersPerSecond + minS * dy,
                        prevSetpoint.robotRelativeSpeeds().omegaRadiansPerSecond + minS * dtheta);
        retSpeeds = ChassisSpeeds.discretize(retSpeeds, dt);

        SwerveModuleState[] retStates = kinematics.toSwerveModuleStates(retSpeeds);
        for (int m = 0; m < numModules; m++) {
            final var maybeOverride = overrideSteering.get(m);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(retStates[m].angle.unaryMinus().rotateBy(override))) {
                    retStates[m].speedMetersPerSecond *= -1.0;
                }
                retStates[m].angle = override;
            }
            final var deltaRotation =
                    prevSetpoint.moduleStates()[m].angle.unaryMinus().rotateBy(retStates[m].angle);
            if (flipHeading(deltaRotation)) {
                retStates[m].angle = retStates[m].angle.rotateBy(Rotation2d.k180deg);
                retStates[m].speedMetersPerSecond *= -1.0;
            }
        }

        return new SwerveSetpoint(retSpeeds, retStates);
    }

    private static boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
    }

    private static double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    private static double findSteeringMaxS(
            double x0, double y0, double theta0, double x1, double y1, double theta1, double maxDeviation) {
        theta1 = unwrapAngle(theta0, theta1);
        double diff = theta1 - theta0;
        if (Math.abs(diff) <= maxDeviation) {
            return 1.0;
        }

        double target = theta0 + Math.copySign(maxDeviation, diff);

        double sin = Math.sin(-target);
        double cos = Math.cos(-target);
        double h0 = sin * x0 + cos * y0;
        double h1 = sin * x1 + cos * y1;

        // Guaranteed not to divide by zero: h0 == h1 would mean theta0 == theta1, already
        // handled above.
        return h0 / (h0 - h1);
    }

    private static boolean isValidS(double s) {
        return Double.isFinite(s) && s >= 0 && s <= 1;
    }

    private static double findDriveMaxS(double x0, double y0, double x1, double y1, double maxVelStep) {
        double l0 = x0 * x0 + y0 * y0;
        double l1 = x1 * x1 + y1 * y1;
        double sqrtL0 = Math.sqrt(l0);
        double diff = Math.sqrt(l1) - sqrtL0;
        if (Math.abs(diff) <= maxVelStep) {
            return 1.0;
        }

        double target = sqrtL0 + Math.copySign(maxVelStep, diff);
        double p = x0 * x1 + y0 * y1;

        double a = l0 + l1 - 2 * p;
        double b = 2 * (p - l0);
        double c = l0 - target * target;
        double root = Math.sqrt(b * b - 4 * a * c);

        double s1 = (-b + root) / (2 * a);
        if (isValidS(s1)) {
            return s1;
        }
        double s2 = (-b - root) / (2 * a);
        if (isValidS(s2)) {
            return s2;
        }

        // Should be unreachable given the check above, but don't limit movement if it happens.
        return 1.0;
    }

    private static boolean epsilonEquals(double a, double b) {
        return Math.abs(a - b) < kEpsilon;
    }

    private static boolean epsilonEquals(ChassisSpeeds s1, ChassisSpeeds s2) {
        return epsilonEquals(s1.vxMetersPerSecond, s2.vxMetersPerSecond)
                && epsilonEquals(s1.vyMetersPerSecond, s2.vyMetersPerSecond)
                && epsilonEquals(s1.omegaRadiansPerSecond, s2.omegaRadiansPerSecond);
    }
}