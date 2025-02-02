package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private SparkMax primaryMotor = new SparkMax(1, MotorType.kBrushless);
    private SparkMax secondaryMotor = new SparkMax(2, MotorType.kBrushless);

    public Intake() {
        var secondaryMotorConfig = new SparkMaxConfig();
        secondaryMotorConfig.follow(this.primaryMotor);

        secondaryMotor.configure(
            secondaryMotorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
    }

    public Command algaeInCommand() {
        return this.startEnd(
            () -> this.primaryMotor.set(-0.25),
            () -> this.primaryMotor.stopMotor());
    }

    public Command algaeOutCommand() {
        return this.startEnd(
            () -> this.primaryMotor.set(0.7),
            () -> this.primaryMotor.stopMotor());
    }
}
