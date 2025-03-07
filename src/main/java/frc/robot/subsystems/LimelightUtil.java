package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class LimelightUtil extends SubsystemBase {
  private final String networkTableName;

  public LimelightUtil(final String networkTableName) {
    this.networkTableName = networkTableName;
  }

  @Override
  public void periodic() { }

  /*Calculates the distance between the center of a detected Limelight object and the camera*/
  public Distance calcDistance() {
    double x = this.getTA();
    
    // Trendline equation generated using Google Sheets- x: tA, y: ft
    // ex: 
    // 
    // +-------+-----+
    // | tA    | ft  |
    // +-------+-----+
    // | 2.958 | 3   |
    // | 2.301 | 3.5 |
    // | 1.544 | 4   |
    // | 1.266 | 4.5 |
    // | 0.951 | 5   |
    // +-------+-----+
    double distanceEquation = 5.13 * Math.pow(x, -0.486); 

    // Arbitrary value derived from imperical testing
    double averageError = (-0.04 * x) - 0.12;

    return Feet.of(distanceEquation + averageError);
  }

  /*Returns the tracked object's middle x-coordinate*/
  public double getTX() {
    return LimelightHelpers.getTX(this.networkTableName);
  }

  /*Returns the tracked object's middle y-coordinate*/
  public double getTY() {
    return LimelightHelpers.getTY(this.networkTableName);
  }

  /*Returns the tracked object's total area*/
  public double getTA() {
    return LimelightHelpers.getTA(this.networkTableName);
  }

  /*Returns the tracked object's ID*/
  public double getID() { 
    return LimelightHelpers.getFiducialID(this.networkTableName);
  }

  /*Returns true if Limelight is detecting a valid object*/
  public boolean isDetecting() { 
    return LimelightHelpers.getTV(this.networkTableName);
  }
}