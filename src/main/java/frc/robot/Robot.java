// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Optional<Command> m_autonomousCommand = Optional.empty();
    private RobotContainer m_robotContainer = RobotContainer.getInstance();

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        //static std::shared_ptr<t34::SwerveDrive> drive = m_this.m_robotContainer.SwerveDrive;
        
        SmartDashboard.putNumber("_Yaw", Gyro.get().getAngle());
    
        this.m_robotContainer.getSwerveDrive().putTelemetry();
        SmartDashboard.putData("Auto chooser: ", this.m_robotContainer.getPathChooser());    
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        Gyro.get().zeroYaw();

        m_autonomousCommand = Optional.ofNullable(this.m_robotContainer.getAutonomousCommand());

        // schedule the autonomous command (example)
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.schedule();
        // }
        m_autonomousCommand.ifPresent(cmd -> cmd.schedule());
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        // if (m_autonomousCommand != null) {
        //     m_autonomousCommand.cancel();
        // }
        m_autonomousCommand.ifPresent(cmd -> cmd.cancel());

        CommandScheduler.getInstance().setDefaultCommand(
            this.m_robotContainer.getSwerveDrive(),
            this.m_robotContainer.getDefaultCommand());
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // PROCESS CONTROLLER BUTTONS
        // Buttons are implemented this way out of simplicity.
        // Consider using button trigger events with commands instead.
    
        // Assign Back Button to Faris Mode.
        if (this.m_robotContainer.getController().getBackButtonReleased()) {
            this.m_robotContainer.getSwerveDrive().toggleFarisMode();
        }
    
        // Assign Start Button to Zeroing Yaw.
        // Note: This is for emergency use only!
        // The robot should be oriented with the front pointed 
        // at the opposite end of the field and sides as 
        // parallel as possible to the fields sides when this
        // button is pressed/released.
        if (this.m_robotContainer.getController().getStartButtonReleased()) {
            Gyro.get().zeroYaw();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
