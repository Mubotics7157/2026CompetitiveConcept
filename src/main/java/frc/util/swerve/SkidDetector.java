package frc.util.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Detects wheel skid by checking how well the four measured module velocities agree with each
 * other as a single rigid body.
 *
 * <p>{@link SwerveDriveKinematics} is overdetermined for 4 modules (3 chassis degrees of freedom,
 * 4 independent wheel measurements), so running the measured module states through
 * {@code kinematics.toChassisSpeeds(...)} gives the least-squares "consensus" chassis motion,
 * and converting that back to module states gives what each module's velocity *should* be if the
 * robot were moving rigidly with no slip. The gap between a module's actual measured velocity and
 * that consensus velocity is, by construction, the part of its motion that isn't explained by
 * rigid-body motion - i.e. slip. A module that's skidding will disagree with the other three; a
 * module that's just doing its job will not.
 */
public class SkidDetector {
    private final SwerveDriveKinematics kinematics;
    private final double skidToleranceMPS;

    private double skidRatio = 0.0;
    private boolean skidding = false;

    /**
     * @param kinematics Kinematics built from the same module locations/order as the drivetrain
     *     reports module states in.
     * @param skidToleranceMPS How much a single module's velocity may disagree with the fitted
     *     consensus, in meters per second, before it's called a skid. Start around 0.3-0.5 m/s and
     *     tune against the real robot - too low will false-trigger on normal scrub while turning.
     */
    public SkidDetector(SwerveDriveKinematics kinematics, double skidToleranceMPS) {
        this.kinematics = kinematics;
        this.skidToleranceMPS = skidToleranceMPS;
    }

    /**
     * Update the skid estimate from the latest measured module states.
     *
     * @param measuredStates Current measured module states, in the same order used to build the
     *     kinematics object.
     * @return true if the wheels currently appear to be skidding.
     */
    public boolean update(SwerveModuleState[] measuredStates) {
        ChassisSpeeds fittedSpeeds = kinematics.toChassisSpeeds(measuredStates);
        SwerveModuleState[] fittedStates = kinematics.toSwerveModuleStates(fittedSpeeds);

        double maxResidualMPS = 0.0;
        double avgSpeedMPS = 0.0;
        for (int m = 0; m < measuredStates.length; m++) {
            double measuredVx = measuredStates[m].angle.getCos() * measuredStates[m].speedMetersPerSecond;
            double measuredVy = measuredStates[m].angle.getSin() * measuredStates[m].speedMetersPerSecond;
            double fittedVx = fittedStates[m].angle.getCos() * fittedStates[m].speedMetersPerSecond;
            double fittedVy = fittedStates[m].angle.getSin() * fittedStates[m].speedMetersPerSecond;

            double residual = Math.hypot(measuredVx - fittedVx, measuredVy - fittedVy);
            maxResidualMPS = Math.max(maxResidualMPS, residual);
            avgSpeedMPS += Math.abs(measuredStates[m].speedMetersPerSecond);
        }
        avgSpeedMPS /= measuredStates.length;

        // Normalize by how fast the robot is actually moving so this reads as a unitless "how bad
        // is it" ratio instead of a raw speed - padded so it doesn't blow up near a standstill.
        skidRatio = maxResidualMPS / Math.max(avgSpeedMPS, 0.1);
        skidding = maxResidualMPS > skidToleranceMPS;
        return skidding;
    }

    /** True if the most recent {@link #update} call detected a skid. */
    public boolean isSkidding() {
        return skidding;
    }

    /** Higher is worse. ~0 means the wheels are rolling together with no disagreement. */
    public double getSkidRatio() {
        return skidRatio;
    }
}