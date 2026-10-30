package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("rio");
    public static final CANBus kCANivoreCANBus = new CANBus("rio");

    // Talon FX IDs
    public static final int kIntakePivot = 21;
    public static final int kIntakeRollers = 22;
    public static final int kFloor = 31;
    public static final int kFeeder = 32;
    public static final int kShooterLeft = 41;
    public static final int kShooterMiddle = 42;
    public static final int kShooterRight = 43;
    public static final int kHanger = 12;

    // PWM Ports
    public static final int kHoodLeftServo = 7;
    public static final int kHoodRightServo = 4;
}
