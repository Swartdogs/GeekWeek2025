// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.SetArmMotorSpeed;
import frc.robot.commands.SetRollerSpeed;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    // First, define the subsystem
    private final Arm _arm = new Arm();

    // Next, define the commands for running the intake rollers in and out
    private final SetRollerSpeed _runRollersIn; // Initialize this here!

    // Lastly, define the commands for raising and lowering the arm

    // The robot's controllers are defined here...
    private final CommandXboxController _controller; // Initialize this here!

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define when commands are run. This will usually
     * be done when some condition is met or state is changed somewhere in
     * the robot or controls (for example, a button is pressed, a sensor
     * reads a specific value, etc.)
     */
    private void configureBindings()
    {
        // While the right bumper is held, run the rollers inward
        _controller.rightBumper().whileTrue(_runRollersIn);

        // While the left bumper is held, run the rollers outward

        // While the right trigger is at least half pressed, raise the arm

        // While the left trigger is at least half pressed, lower the arm
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // "null" indicates no autonomous will be run
        return null;
    }
}
