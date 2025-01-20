package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TrajMath {
    private double m_note_max_velocity_mps;
    private double m_motor_output;
    private double m_target_distance_meters;
    private double m_target_height_meters;
    private double m_apriltag_height_meters;
    private double m_limelight_height_meters;
    private double m_shooter_angle_degrees;
    private double m_limelight_angle_degrees;

    private double m_target_tx;
    private double m_target_ty;

    private double m_previous_firing_angle;

    private static final double g = 9.80665; // gravity

    public TrajMath(
        final double note_max_velocity_mps,
        final double target_height_meters,
        final double apriltag_height_meters,
        final double limelight_height_meters,
        final double shooter_angle,
        final double limelight_angle
    ) {
        this.m_note_max_velocity_mps   = note_max_velocity_mps;
        this.m_target_height_meters    = target_height_meters;
        this.m_apriltag_height_meters  = apriltag_height_meters;
        this.m_limelight_height_meters = limelight_height_meters;
        this.m_shooter_angle_degrees   = shooter_angle;
        this.m_limelight_angle_degrees = limelight_angle;
        this.m_target_tx               = 0.0;
        this.m_target_ty               = 0.0;
        this.m_previous_firing_angle   = 90.0;
    }

    public void periodic() {
        this.m_target_ty = LimelightHelpers.getTY(Constants.LIMELIGHT_TABLE_NAME);
        this.m_target_tx = LimelightHelpers.getTX(Constants.LIMELIGHT_TABLE_NAME);
        this.m_limelight_angle_degrees = this.m_target_ty * Constants.LIMELIGHT_DEGREE_SCALAR;    
    }

    public void putTelemetry() {
        SmartDashboard.putNumber("Limelight degree: ", this.m_limelight_angle_degrees);
        SmartDashboard.putNumber("Target height: ", (this.m_target_height_meters - this.m_limelight_height_meters) );
        SmartDashboard.putNumber("Arm Firing Angle (degrees): ", getArmFiringAngleDeg());
        SmartDashboard.putNumber("TrajMath tx: ", this.m_target_tx);
        SmartDashboard.putNumber("TrajMath ty: ", this.m_target_ty);
    }

    public double getArmFiringAngleDeg() {
        final double x = this.getDistanceFromTarget();
        final double x2 = x * x;
        double firingAngleDeg = (-1.9171 * x2) + (17.186 * x) + (3.5412);

        if (Double.isNaN(firingAngleDeg)) {
            return this.m_previous_firing_angle;
        }

        this.m_previous_firing_angle = firingAngleDeg;

        return firingAngleDeg;
    }

    public boolean isInRange() {
        final double theta = Math.toRadians(this.m_shooter_angle_degrees);
        final double v2 = this.m_note_max_velocity_mps * this.m_note_max_velocity_mps;
        final double x = this.m_target_distance_meters;
        final double cosTheta = Math.cos(theta);
        final double cos2Theta = cosTheta * cosTheta;
    
        final double rate_of_rise = -(((2 * g) / (2 * v2 * cos2Theta)) * x) + Math.tan(theta);
    
        return rate_of_rise > 0; 
    }

    public double getDistanceFromTarget() {
        //return (this.m_target_height_meters - this.m_limelight_height_meters) / Math.tan( Math.toRadians(this.m_limelight_angle_degrees) )
        return Maths.log2(2.5 / LimelightHelpers.getTA(Constants.LIMELIGHT_TABLE_NAME)) - 0.3;
    }

    public void inputMotorOutputPercent(final double percent) {
        this.m_motor_output = percent;
    }

    public void setTargetHeightMeters(final double meters) {
        this.m_target_height_meters = meters;
    }

    public void setAprilTagHeightMeters(final double meters) {
        this.m_apriltag_height_meters = meters;
    }
}
