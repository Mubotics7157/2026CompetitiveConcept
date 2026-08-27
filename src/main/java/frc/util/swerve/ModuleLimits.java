package frc.util.swerve;

/**
 * Kinematic limits fed into the {@link SwerveSetpointGenerator}. These describe what a single
 * module is physically capable of, not what the driver is asking for - the generator uses them
 * to figure out how much of the requested motion can actually be achieved this loop without a
 * wheel needing to teleport.
 *
 * @param maxDriveVelocityMPS Maximum wheel speed, in meters per second.
 * @param maxDriveAccelerationMPSSq Maximum wheel speed change per second, in meters per second
 *     squared. Lower this to make the robot ramp up/down more gently.
 * @param maxSteeringVelocityRadPerSec Maximum azimuth (steering) turn rate, in radians per
 *     second.
 */
public record ModuleLimits(
        double maxDriveVelocityMPS,
        double maxDriveAccelerationMPSSq,
        double maxSteeringVelocityRadPerSec) {}