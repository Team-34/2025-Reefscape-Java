package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final class Neo550 {
    private static final double   RESOLUTION    = 42.0;
    private static final Distance CIRCUMFERENCE = Inches.of((1/8f) * Math.PI);

    public static double angleToNativeUnits(Angle angle) {
        return angle.in(Rotations) * RESOLUTION;
    }

    public static double distanceToNativeUnits(Distance distance) {
        return distance.div(CIRCUMFERENCE).in(Value) * RESOLUTION;
    }
}
