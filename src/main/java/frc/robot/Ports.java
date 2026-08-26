package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("swerve");

    // Talon FX IDs
    public static final int kIntakePivot = 14;
    public static final int kIntakeRollers = 11;
    public static final int kFloor = 19;
    public static final int kFeeder = 27;
    public static final int kShooterLeft = 24;
    public static final int kShooterMiddle = 16;
    public static final int kShooterRight = 20;
    public static final int kHanger = 12;

    // PWM Ports
    public static final int kHoodLeftServo = 0;
    public static final int kHoodRightServo = 1;
}
