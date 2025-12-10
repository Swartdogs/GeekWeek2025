// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.IntakeGamePiece;
import frc.robot.commands.PlaceGamePiece;
import frc.robot.commands.SetArmMotorSpeed;
import frc.robot.commands.SetArmAngle;
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
    private final Arm _arm = new Arm();

    private final SetRollerSpeed _runRollersIn = new SetRollerSpeed(_arm, Constants.ROLLER_IN_SPEED);
    private final SetRollerSpeed _runRollersOut = new SetRollerSpeed(_arm, Constants.ROLLER_OUT_SPEED);

    // We don't need these commands anymore since the PID controller will handle this for us. Comment them out for now
    private final SetArmMotorSpeed _raiseArm = new SetArmMotorSpeed(_arm, 0.3);
    private final SetArmMotorSpeed _lowerArm = new SetArmMotorSpeed(_arm, -0.3);

    // Create two commands for setting the arm angle. One will set the arm to the high angle, the other will
    // set it to the low angle

    // Create commands for intaking and placing game pieces

    // The robot's controllers are defined here...
    private final CommandXboxController _controller = new CommandXboxController(0);

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
        _controller.rightBumper().whileTrue(_runRollersIn);
        _controller.leftBumper().whileTrue(_runRollersOut);

        // Re-bind the triggers so the left trigger sets the arm to the low position
        // and the right trigger sets the arm to the high position
        _controller.rightTrigger(0.5).whileTrue(_raiseArm);
        _controller.leftTrigger(0.5).whileTrue(_lowerArm);

        // Button A should run the "intake game piece" command while the button is held

        // Button B should run the "place game piece" command while the button is held
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
