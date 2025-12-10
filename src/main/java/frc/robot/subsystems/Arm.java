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

    private final PIDController _armAnglePID = new PIDController(Constants.ARM_ANGLE_KP, Constants.ARM_ANGLE_KI, Constants.ARM_ANGLE_KD);

    public Arm()
    {
        _straightWheelMotor.setInverted(true);
        _angledWheelMotor.setInverted(true);

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
        // _armAngleMotor.set(speed);
    }

    public void setArmAngle(double angle)
    {
        _armAnglePID.setSetpoint(angle);
    }

    @Override
    public void periodic()
    {
        double angle = _armAngleMotor.getPosition().getValueAsDouble();
        double speed = _armAnglePID.calculate(angle);

        System.out.println(String.format("Target: %5.2f, Actual: %5.2f", _armAnglePID.getSetpoint(), angle));

        _armAngleMotor.set(speed);
    }
}
