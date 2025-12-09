// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetArmMotorSpeed extends Command
{
    // Things we need in order to execute this command
    private final Arm _arm;

    /**
     * Creates a new SetRollerSpeed.
     *
     * @param arm The subsystem used by this command.
     * @param speed The speed to run the arm angle motor at
     */
    public SetArmMotorSpeed(Arm arm, double speed)
    {
        _arm = arm;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        // Set the arm motor to the desired speed
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
        // Turn off the arm motor
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
