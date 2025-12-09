// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// This class defines all of the hardware that makes up the Arm
// subsystem and how to interact with it
public class Arm extends SubsystemBase
{
    // Motors and other hardware are defined here
    // private final VictorSP _straightWheelMotor = new VictorSP(0);

    // Initialize motor and sensor settings here
    public Arm()
    {

    }

    // This function sets both wheels on the intake to the same speed
    public void setRollerSpeed(double percentOutput)
    {

    }

    // This function sets the arm angle motor to a given speed
    public void setArmMotorSpeed(double percentOutput)
    {

    }
}
