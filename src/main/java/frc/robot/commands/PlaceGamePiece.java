// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class PlaceGamePiece extends SequentialCommandGroup
{
    /** Creates a new PlaceGamePiece. */
    public PlaceGamePiece(Arm arm)
    {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
            // Steps:
            //  1. Set the arm angle to "high"
            //  2. Set the roller speed to the "out speed"
            //  3. Set the arm angle to "rest"
        );
    }
}
