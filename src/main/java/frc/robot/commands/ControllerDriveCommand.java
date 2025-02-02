package frc.robot.commands;

import java.time.Duration;
import java.time.Instant;
import java.time.InstantSource;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;

public class ControllerDriveCommand extends Command {
    private final SwerveDrive drive;
    private final CommandXboxController controller;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;
    private final SlewRateLimiter rLimiter;

    private final InstantSource clock;
    private Instant m_last_zero = null;

    public ControllerDriveCommand(SwerveDrive drive, CommandXboxController controller, InstantSource clock) {
        this.drive = drive;
        this.controller = controller;

        this.xLimiter = new SlewRateLimiter(1.1);
        this.yLimiter = new SlewRateLimiter(1.1);
        this.rLimiter = new SlewRateLimiter(2.0);
        
        this.clock = clock;
        this.m_last_zero = clock.instant();

        this.addRequirements(drive);
    }

    @Override
    public void execute() {
        final double x   = this.xLimiter.calculate(this.controller.getLeftX());
        final double y   = this.yLimiter.calculate(this.controller.getLeftY());
        final double rot = this.rLimiter.calculate(this.controller.getRightX());

        if (isInputZero(x, y, rot)) {
            this.drive.stop();

            // When there is no input and when the alloted time has elapsed, rezero the swerve wheels.
            // The alloted time is in seconds and can be set using the ZERO_SWERVE_TIME_SECONDS constexpr
            // located in SwerveContants.h. Suggested value is 5 seconds.
            final Instant now = this.clock.instant();
            final Duration elapsed = Duration.between(this.m_last_zero, now);
            if (elapsed.compareTo(SwerveConstants.ZERO_SWERVE_TIME) >= 0) {
                this.m_last_zero = now;
                this.drive.resetToAbsolute();
            }  

            return;
        }
        
        double xSpeed = Math.copySign(scaleToRange(-(x * x), 0.0, 1.0, 0.0, SwerveConstants.DRIVE_MAX_SPEED), x);
        double ySpeed = Math.copySign(scaleToRange(-(y * y), 0.0, 1.0, 0.0, SwerveConstants.DRIVE_MAX_SPEED), y);
        double rSpeed = Math.copySign(scaleToRange(-(rot * rot), 0.0, 1.0, 0.0, SwerveConstants.STEER_MAX_SPEED), rot);

        this.drive.drive(new Translation2d(xSpeed, ySpeed), rSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.drive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static double scaleToRange(
        final double x, 
        final double inMin, 
        final double inMax, 
        final double outMin, 
        final double outMax)
    {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    private boolean isInputZero(final double x, final double y, final double r) {
        final double posTolerance =  0.00001;
        final double negTolerance = -0.00001;

        double combined = x + y + r;
        return combined < posTolerance && combined > negTolerance;
    }
}
