package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(15.0);
    public static final double MAX_SPEED2 = Units.feetToMeters(14.5/10.0);

    public static final double DRIVER_DEADBAND = 0.3;
    public static final int ELEVATOR_LIMITSWITCH_CHANNEL_TOP = 0;
    public static final int ELEVATOR_LIMITSWITCH_CHANNEL_BOTTOM = 1;

}