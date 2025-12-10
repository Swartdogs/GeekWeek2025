// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase
{
    private final VictorSP _straightWheelMotor = new VictorSP(0);
    private final VictorSP _angledWheelMotor = new VictorSP(1);

    private final TalonFX _armAngleMotor = new TalonFX(1);

    // Initialize the PID controller using the coefficients in the "Constants" class.
    private final PIDController _armAnglePID;

    public Arm()
    {
        _straightWheelMotor.setInverted(true);
        _angledWheelMotor.setInverted(true);

        // Previously, we had a really long line to tell the motor to be inverted. We have some other things to set, and we can do
        // all of it together like this:
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.PeakForwardDutyCycle = Constants.MAX_ARM_ANGLE_MOTOR_SPEED;
        configs.MotorOutput.PeakReverseDutyCycle = -Constants.MAX_ARM_ANGLE_MOTOR_SPEED;
        configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        _armAngleMotor.getConfigurator().apply(configs);
        _armAngleMotor.setPosition(0);
    }

    public void setRollerSpeed(double percentOutput)
    {
        _straightWheelMotor.set(percentOutput);
        _angledWheelMotor.set(percentOutput);
    }

    public void setArmMotorSpeed(double speed)
    {
        // We'll be using a PID controller, so we don't need this function anymore.
        // To prevent breaking things, we'll just comment out this line
        _armAngleMotor.set(speed);
    }

    public void setArmAngle(double angle)
    {
        // Tell the PID controller to go to a new setpoint
    }

    @Override
    public void periodic()
    {
        // The PID controller will continuously run here. To make this work we need to:
        //  1. get the current arm position
        //  2. tell the PID controller to calculate a new speed
        //  3. print out what our current target and position are
        //  4. set the speed of the motor based on what the PID controller calculated
    }
}
