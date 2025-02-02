// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ControllerDriveCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;

import java.time.Clock;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final ControllerDriveCommand defaultCommand;

    private final Clock clock = Clock.systemUTC();

    private final CommandXboxController ctrl;
    private final SwerveDrive swerveDrive;
    private final Climber climber;
    private final Intake intake;


    // // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

    // // Replace with CommandPS4Controller or CommandJoystick if needed
    // private final CommandXboxController m_driverController =
    //     new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        this.swerveDrive = new SwerveDrive();
        this.ctrl = new CommandXboxController(0);
        this.defaultCommand = new ControllerDriveCommand(this.swerveDrive, this.ctrl, this.clock);
        this.climber = new Climber();
        this.intake = new Intake();
        
        // Configure the trigger bindings
        this.configureBindings();

    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        //Runs algae intake in on A button, spits out on B
        this.ctrl.a().onTrue(intake.algaeInCommand());
        this.ctrl.b().onTrue(intake.algaeOutCommand());

        //Moves the climber up and down on Left Bumper press
        this.ctrl.leftBumper().onTrue(climber.flipArmCommand());
    }

    public CommandXboxController getController()     { return this.ctrl; }
    public SwerveDrive           getSwerveDrive()    { return this.swerveDrive; }
    public Command               getDefaultCommand() { return this.defaultCommand; }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);

        return new InstantCommand(() -> {
          this.swerveDrive.drive(new Translation2d(0.0, -1.0), 0.0);
        }, this.swerveDrive);

        // final String path_file_name = this.path_chooser.getSelected();
        // final PathPlannerPath path = PathPlannerPath.fromPathFile(path_file_name);
    
        // return AutoBuilder.followPath(path);
    }
}
