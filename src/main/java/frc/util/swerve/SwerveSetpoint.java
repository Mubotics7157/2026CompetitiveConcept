package frc.util.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A commanded drivetrain state: robot-relative chassis speeds plus the exact per-module states
 * that produce them. The {@link SwerveSetpointGenerator} steps one of these toward a desired
 * state each loop, and hands back the next one to command and to remember as "previous" for the
 * following loop.
 */
public record SwerveSetpoint(ChassisSpeeds robotRelativeSpeeds, SwerveModuleState[] moduleStates) {

    /** A setpoint with zero speed at each module's current angle - safe to start/reset from. */
    public static SwerveSetpoint zero(SwerveModuleState[] currentStates) {
        SwerveModuleState[] states = new SwerveModuleState[currentStates.length];
        for (int i = 0; i < states.length; i++) {
            states[i] = new SwerveModuleState(0.0, currentStates[i].angle);
        }
        return new SwerveSetpoint(new ChassisSpeeds(), states);
    }
}