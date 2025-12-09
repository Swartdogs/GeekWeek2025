// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

/**
 * Central hub for robot configuration.
 * Creates subsystems, controllers, binds buttons to commands.
 *
 * Controller Button Mapping:
 * - Right Trigger (RT): Raise arm
 * - Left Trigger (LT): Lower arm
 * - Right Bumper (RB): Intake wheels in
 * - Left Bumper (LB): Intake wheels out
 * - A Button: Intake position preset
 * - B Button: Stow position preset
 */
public class RobotContainer
{
    // Subsystems
    private final Arm _arm = new Arm();
    private final Intake _intake = new Intake();

    // Controller (port 0 = first controller in Driver Station)
    private final CommandXboxController _controller =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /**
     * Creates the RobotContainer and configures button bindings.
     */
    public RobotContainer()
    {
        configureBindings();
    }

    /**
     * Configures button-to-command bindings.
     * whileTrue() - runs while button held, stops when released
     * onTrue() - runs once when button pressed
     */
    private void configureBindings()
    {
        // Manual arm controls
        _controller.rightTrigger(0.5).whileTrue(_arm.raiseCommand());
        _controller.leftTrigger(0.5).whileTrue(_arm.lowerCommand());
        
        // Manual intake controls
        _controller.rightBumper().whileTrue(_intake.intakeCommand());
        _controller.leftBumper().whileTrue(_intake.outtakeCommand());

        // Preset positions
        _controller.a().onTrue(createIntakePositionCommand());
        _controller.b().onTrue(createStowPositionCommand());
    }

    /**
     * Creates intake position command (sequence).
     * 1. Move arm to intake angle
     * 2. Run intake wheels
     */
    private Command createIntakePositionCommand()
    {
        return Commands.sequence(
            _arm.goToIntakeCommand()
                .withTimeout(2.0),
            _intake.intakeCommand()
        ).withName("Intake Position");
    }
    
    /**
     * Creates stow position command (parallel).
     * Simultaneously moves arm to stow and stops wheels.
     */
    private Command createStowPositionCommand()
    {
        return Commands.parallel(
            _arm.goToStowCommand(),
            _intake.stopCommand()
        ).withName("Stow Position");
    }

    /**
     * Gets autonomous command (currently none for GeekWeek).
     */
    public Command getAutonomousCommand()
    {
        return Commands.none();
    }
    
    // Subsystem accessors for testing/debugging
    
    public Arm getArm()
    {
        return _arm;
    }
    
    public Intake getIntake()
    {
        return _intake;
    }
}
