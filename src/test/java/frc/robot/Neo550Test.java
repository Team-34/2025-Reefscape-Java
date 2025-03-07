package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.params.provider.Arguments.arguments;

import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Neo550Test {
    static Stream<Arguments> angleToNativeUnits_testParams() {
        return Stream.of(
            arguments(Degrees.of(90), 10.5),
            arguments(Rotations.of(0.5), 21.0),
            arguments(Degrees.of(270), 31.5),
            arguments(Rotation.one(), 42.0),
            arguments(Rotations.of(2), 84.0)
        );
    }

    @ParameterizedTest
    @MethodSource("angleToNativeUnits_testParams")
    void angleToNativeUnits_convertsAnAngleToNativeNeo550Units(Angle angle, double expected) {
        assertEquals(expected, Neo550.angleToNativeUnits(angle));
    }

    static Stream<Arguments> distanceToNativeUnits_testParams() {
      return Stream.of(
          arguments(Inches.of((1/8f) * Math.PI).div(4), 10.5),
          arguments(Inches.of((1/8f) * Math.PI).div(2), 21.0),
          arguments(Inches.of((1/8f) * Math.PI).times(0.75), 31.5),
          arguments(Inches.of((1/8f) * Math.PI), 42.0),
          arguments(Inches.of((1/8f) * Math.PI).times(1.5), 63.0),
          arguments(Inches.of((1/8f) * Math.PI).times(2), 84.0)
      );
  }

  @ParameterizedTest
  @MethodSource("distanceToNativeUnits_testParams")
  void distanceToNativeUnits_convertsAnDistanceToNativeNeo550Units(Distance distance, double expected) {
      assertEquals(expected, Neo550.distanceToNativeUnits(distance));
  }
}
