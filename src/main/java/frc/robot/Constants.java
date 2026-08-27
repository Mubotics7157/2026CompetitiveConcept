// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        public static final AngularVelocity kPIDRotationDeadband = kMaxRotationalRate.times(0.005);
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    /**
     * Values used by the swerve setpoint generator and skid detector to keep the drivetrain from
     * asking for more grip than the wheels actually have. These are starting-point estimates -
     * weigh the real robot for kMassKg, and tune kWheelCoefficientOfFriction/kSkidToleranceMPS
     * against how the real wheels behave on the actual carpet before relying on them.
     */
    public static class Traction {
        /** Update to the weighed robot mass (with bumpers and battery). */
        public static final double kMassKg = 18.1436948; // ~40 lb TODO:CHANGE THIS LATER WHEN ROBOT GETS BUILT ON

        /** Roughly 1.0-1.5 for treaded wheels on carpet; lower for slicker wheels/surfaces. */
        public static final double kWheelCoefficientOfFriction = 1.1;

        public static final double kMaxLinearAccelerationMPSSq = 8.0;
        public static final AngularVelocity kMaxSteeringVelocity = RotationsPerSecond.of(6);

        /** How much to derate acceleration while a skid is detected, to help the wheels recover grip. */
        public static final double kSkidAccelerationScale = 0.5;

        /** Per-module velocity disagreement, in m/s, above which it's called a skid. */
        public static final double kSkidToleranceMPS = 0.4;
    }
}