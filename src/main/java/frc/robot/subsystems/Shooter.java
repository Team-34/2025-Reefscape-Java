package frc.robot.subsystems;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Maths;

public class Shooter {
    //double m_arm_angle_top{};
    //double m_arm_angle_bottom{};

    private double m_max_speed_percent  = 0.8;
    private double m_arm_angle_setpoint = 90.0;

    private boolean arm_using_pid = true;

    private SparkMax m_firing_motor_left    = null;
    private SparkMax m_firing_motor_right   = null;

    private SparkMax m_arm_motor_top          = null;
    private SparkMax m_arm_motor_bottom       = null;

    private SparkClosedLoopController m_arm_pidctrl_top    = null;
    private SparkClosedLoopController m_arm_pidctrl_bottom = null;

    private TalonSRX m_intake_motor = null; // Only supported in Phoenix 5

    //     //frc::PIDController m_arm_pid;

    private DigitalInput m_note_sensor = null;
    private DigitalInput m_arm_sensor  = null;

    private InstantSource clock = null;
    private Instant m_current_time = null;
    private Instant m_since_run_shooter = null;
    private Duration m_time_delta = null;
    private boolean m_reset_time = false;

    public Shooter(final InstantSource clock) {

        {
            var config = new SparkMaxConfig();
            config.smartCurrentLimit(30);
            
            this.m_firing_motor_left = new SparkMax(14, MotorType.kBrushless);
            this.m_firing_motor_left.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        
        {
            var config = new SparkMaxConfig();
            config.smartCurrentLimit(30);

            this.m_firing_motor_right = new SparkMax(11, MotorType.kBrushless);
            this.m_firing_motor_right.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        {
            var config = new SparkMaxConfig();
            config.smartCurrentLimit(20);
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.5, 0.0, 0.05);
            config.encoder
                .positionConversionFactor(Constants.ARM_ENC_CONVERSION_FACTOR);

            this.m_arm_motor_top = new SparkMax(2, MotorType.kBrushless);
            this.m_arm_motor_top.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        {
            var config = new SparkMaxConfig();
            config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.5, 0.0, 0.05);
            config.encoder.positionConversionFactor(Constants.ARM_ENC_CONVERSION_FACTOR);
            config.smartCurrentLimit(20);

            this.m_arm_motor_bottom = new SparkMax(1, MotorType.kBrushless);
            this.m_arm_motor_bottom.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }

        this.m_intake_motor = new TalonSRX(12);
        
        this.m_note_sensor = new DigitalInput(9);
        this.m_arm_sensor  = new DigitalInput(8);

        this.clock = clock;
        this.m_current_time = clock.instant();
        this.m_since_run_shooter = clock.instant();
        this.m_time_delta = Duration.ZERO;
        this.m_reset_time = false;
        
        this.m_arm_motor_top.getEncoder().setPosition(2.4803999);
        this.m_arm_motor_bottom.getEncoder().setPosition(this.m_arm_motor_top.getEncoder().getPosition());
    }

    public void runShooterPercent(final double motor_output) {
        double speed = Maths.clamp(motor_output, -this.m_max_speed_percent, this.m_max_speed_percent);

        m_firing_motor_left.set(speed);
        m_firing_motor_right.set(speed);
    
        // final boolean is_firing = speed > 0.5;
        // if (is_firing)
        // {
        //     final double left_speed = m_firing_encoder_left.getVelocity();
        //     final double avg_speed = (left_speed + right_speed) / 2;
        
        //     if (avg_speed >= speed)
        //     {
        //         final boolean IGNORE_SENSOR = true;
        //         this.runIntakeMotorPercent(speed, IGNORE_SENSOR);
        //     } 
        // }
    }

    public void runIntakeMotorPercent(final double motor_output) {
        this.runIntakeMotorPercent(motor_output, false);
    }

    public void runIntakeMotorPercent(final double motor_output, final boolean bypass_sensor) {
        double clamp_max = (this.intakeHasNote() && !bypass_sensor) ? 0.0 : 1.0;
        double percent_output = Maths.clamp(motor_output, -1.0, clamp_max);
        this.m_intake_motor.set(ControlMode.PercentOutput, percent_output);
    }

    //public void MoveArmToAngleDeg(final double angle)
    //{
    //    m_arm_angle_setpoint = angle * ARM_DEG_SCALAR;
    //
    //    m_arm_pidctrl_top.SetReference(m_arm_angle_setpoint, rev::ControlType::kPosition);
    //    m_arm_pidctrl_bottom.SetReference(m_arm_angle_setpoint, rev::ControlType::kPosition);
    //    
    //}
    //
    //public void MoveShooterToAngleDeg(final double angle)
    //{
    //    m_arm_angle_setpoint = ((angle - SHOOTER_OFFSET_ANGLE_DEG) * ARM_DEG_SCALAR);
    //
    //    MoveArmToAngleDeg(angle);
    //}

    public double getMaxSpeedPercent() {
        return this.m_max_speed_percent;
    }
    public void setMaxSpeedPercent(final double percent) {
        this.m_max_speed_percent = percent;
    }

    public void configForAmp() {
        this.setSetpoint(87.18);
        this.setMaxSpeedPercent(0.11);
    }

    public void configForSpeaker(final double shooter_firing_angle) {
        this.setSetpoint(shooter_firing_angle);
        this.setMaxSpeedPercent(0.8);
    }

    public void configForRest() {
        this.setSetpoint(90.0);
        this.setMaxSpeedPercent(0.8);    
    }

    public void configForNoteCollection() {
        this.setSetpoint(23.0);
        this.setMaxSpeedPercent(0.8);    
    }
    
    public void periodic() {
        this.m_arm_angle_setpoint = Maths.clamp(this.m_arm_angle_setpoint, 12.0, 90.0);
        this.m_current_time = clock.instant();

        //double motor_output = 
        //(fabs(m_kp * ( ( (m_arm_angle_setpoint / ARM_DEG_SCALAR) - GetTopArmEncoderVal()) / m_arm_angle_setpoint)) < m_tolerance) ? 
        //0.0 : (m_kp *  ( (m_arm_angle_setpoint / ARM_DEG_SCALAR) - GetTopArmEncoderVal()));
    
        if (this.usingPIDArmMovement())
        {
            //m_arm_motor_top.Set(motor_output);
            //m_arm_motor_bottom.Set(motor_output);
            final double rotations = this.m_arm_angle_setpoint * Constants.ARM_DEG_SCALAR;
            this.m_arm_pidctrl_top.setReference(rotations, ControlType.kPosition);
            this.m_arm_pidctrl_bottom.setReference(rotations, ControlType.kPosition);
        }
    }

    public void putTelemetry() {
        SmartDashboard.putNumber("Arm Top Relative Encoder: ", this.getTopArmEncoderVal() / Constants.ARM_DEG_SCALAR);
        SmartDashboard.putNumber("Arm Bottom Relative Encoder: ", this.getBottomArmEncoderVal() / Constants.ARM_DEG_SCALAR);
        
        SmartDashboard.putNumber("Max Speed: ", m_max_speed_percent);
        SmartDashboard.putNumber("Arm Setpoint: ", m_arm_angle_setpoint);
    
        //SmartDashboard.putBoolean("IsIntakeMovingBackwards: ", IsIntakeMovingBackward(m_intake_motor.GetMotorOutputPercent()));
        SmartDashboard.putBoolean("UsingPIDArmMovement: ", this.usingPIDArmMovement());

        SmartDashboard.putNumber("Time delta (seconds)", this.m_time_delta.getSeconds());
    }

    public void setZero() {
        this.m_arm_motor_top.getEncoder().setPosition(0.0);
        this.m_arm_motor_bottom.getEncoder().setPosition(0.0);    
    }

    public void setSetpoint(final double setpoint) {
        this.m_arm_angle_setpoint = setpoint;
    }

    private static Duration SHOOTER_SPINUP_TIME = Duration.ofSeconds(1);
    public void shoot(double motorOutput) {
        final boolean IGNORE_SENSOR = true;

        this.runShooterPercent(motorOutput);

        this.m_time_delta = Duration.between(this.m_current_time, this.m_since_run_shooter);

        final boolean isShooterSpunUp = this.m_time_delta.compareTo(SHOOTER_SPINUP_TIME) > 1;
        if (isShooterSpunUp)
        {
            this.runIntakeMotorPercent(0.7, IGNORE_SENSOR);
        }
        else
        {
            this.runIntakeMotorPercent(0.0, IGNORE_SENSOR);
        }

        this.updateShooterClock();
    }

    private final static Duration MAX_TIME_DELTA = Duration.ofSeconds(2);
    public void updateShooterClock() {
        final boolean shouldUpdateShooterClock = this.m_time_delta.compareTo(MAX_TIME_DELTA) > 0;
        if (shouldUpdateShooterClock) this.m_since_run_shooter = this.clock.instant();
    }

    public void moveUp() { 
        this.m_arm_angle_setpoint += 1.0; 
    }
    public void moveDown() { 
        this.m_arm_angle_setpoint -= 1.0; 
    }

    public double getTopArmEncoderVal() {
        return this.m_arm_motor_top.getEncoder().getPosition();
    }
    public double getBottomArmEncoderVal() { 
        return this.m_arm_motor_bottom.getEncoder().getPosition(); 
    }

    public void runTopArmMotorPercent(final double motor_output) { 
        this.m_arm_motor_top.set(motor_output);
    }
    public void runBottomArmMotorPercent(final double motor_output) {
        this.m_arm_motor_bottom.set(motor_output); 
    }

    public void runLeftFiringMotorPercent(final double motor_output) { 
        this.m_firing_motor_left.set(motor_output); 
    }
    public void runRightFiringMotorPercent(final double motor_output) { 
        this.m_firing_motor_right.set(motor_output); 
    }

    public void togglePIDArmMovement() {
        this.arm_using_pid = !this.arm_using_pid;
    }
    public boolean usingPIDArmMovement() {
        return this.arm_using_pid;
    }

    public boolean intakeHasNote() {
        return this.m_note_sensor.get();
    }

    public boolean isArmAtZero() { 
        return m_arm_sensor.get(); 
    }

    private boolean isIntakeMovingBackward(final double motor_output) {
        return (-motor_output) < 0.0;
    }
}
