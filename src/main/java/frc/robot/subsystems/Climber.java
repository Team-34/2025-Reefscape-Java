package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private static final double ENGAGEMENT_SPEED    =  0.5;
    private static final double DISENGAGEMENT_SPEED = 33.0;

    private SparkMax motor = new SparkMax(5, MotorType.kBrushless);
    private PIDController pidController = new PIDController(0.1, 0.0, 0.0);
    private boolean engaged = false;

    public Climber() {}

    public Command flipArmCommand() {
        return this.runEnd(() -> {
            final var speed = this.pidController.calculate(
                this.motor.getEncoder().getPosition(),
                engaged ? ENGAGEMENT_SPEED : DISENGAGEMENT_SPEED);
            this.motor.set(speed);
        },
        () -> {
            this.engaged = !this.engaged;
            this.motor.stopMotor();
        }).until(() -> this.pidController.atSetpoint());
    }
}
