// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetArmAngle extends Command
{
    private final Arm _arm;

    /**
     * Creates a new SetArmPosition.
     *
     * @param arm The subsystem used by this command.
     */
    public SetArmAngle(Arm arm, double angle)
    {
        _arm = arm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        // Set the arm angle to the provided angle
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted)
    {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        // When is the command finished? When we reach that point, return
        // true
    }
}
