// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase
{
    private final VictorSP _straightWheelMotor = new VictorSP(0);
    private final VictorSP _angledWheelMotor = new VictorSP(1);

    private final TalonFX _armAngleMotor = new TalonFX(1);

    public Arm()
    {
        _straightWheelMotor.setInverted(true);
        _angledWheelMotor.setInverted(true);

        _armAngleMotor.getConfigurator().apply(new TalonFXConfiguration().MotorOutput.withInverted(InvertedValue.Clockwise_Positive));
    }

    public void setRollerSpeed(double percentOutput)
    {
        _straightWheelMotor.set(percentOutput);
        _angledWheelMotor.set(percentOutput);
    }

    public void setArmMotorSpeed(double speed)
    {
        _armAngleMotor.set(speed);
    }
}
