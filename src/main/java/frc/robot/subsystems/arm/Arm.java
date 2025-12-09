// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Controls a single-jointed arm mechanism.
 * Hardware: 2x VictorSPX motors (leader/follower), encoder on roboRIO DIO.
 */
public class Arm extends SubsystemBase
{
    // Motor controllers
    private final WPI_VictorSPX _motor1;  // Leader motor
    private final WPI_VictorSPX _motor2;  // Follower motor

    // Position feedback and control
    private final Encoder _encoder;
    private final PIDController _pidController;
    private double _targetAngle = 0.0;
    private boolean _usingPID = false;

    /**
     * Creates a new Arm subsystem.
     * Configures motors, encoder, and PID controller.
     */
    public Arm()
    {
        // Motor setup
        _motor1 = new WPI_VictorSPX(Constants.CAN.ARM_MOTOR_1);
        _motor2 = new WPI_VictorSPX(Constants.CAN.ARM_MOTOR_2);
        
        _motor1.configFactoryDefault();
        _motor2.configFactoryDefault();
        
        // BRAKE mode helps arm hold position when stopped
        _motor1.setNeutralMode(NeutralMode.Brake);
        _motor2.setNeutralMode(NeutralMode.Brake);
        
        // Motor 2 follows motor 1
        _motor2.follow(_motor1);
        
        // Configure follower inversion based on physical mounting
        if (Constants.Arm.MOTOR_2_INVERTED)
        {
            _motor2.setInverted(InvertType.OpposeMaster);
        }
        else
        {
            _motor2.setInverted(InvertType.FollowMaster);
        }
        
        // Encoder setup: connected to roboRIO DIO (VictorSPX cannot read encoders directly)
        _encoder = new Encoder(Constants.DIO.ARM_ENCODER_A, Constants.DIO.ARM_ENCODER_B);
        _encoder.setDistancePerPulse(Constants.Arm.ENCODER_DISTANCE_PER_PULSE);
        _encoder.reset();
        
        // PID controller for automatic position control
        _pidController = new PIDController(
            Constants.Arm.kP,
            Constants.Arm.kI,
            Constants.Arm.kD
        );
        _pidController.setTolerance(Constants.Arm.POSITION_TOLERANCE);
    }

    /**
     * Called every 20ms. Runs PID control loop if enabled.
     */
    @Override
    public void periodic()
    {
        if (_usingPID)
        {
            double pidOutput = _pidController.calculate(getAngle(), _targetAngle);
            double feedforward = Constants.Arm.kG * Math.cos(Math.toRadians(getAngle()));
            _motor1.set(ControlMode.PercentOutput, pidOutput + feedforward);
        }
    }

    // Manual control methods

    /**
     * Raises the arm at configured UP_SPEED.
     * Call continuously while button is held.
     */
    public void raise()
    {
        _usingPID = false;
        _motor1.set(ControlMode.PercentOutput, Constants.Arm.UP_SPEED);
    }
    
    /**
     * Lowers the arm at configured DOWN_SPEED.
     * Call continuously while button is held.
     */
    public void lower()
    {
        _usingPID = false;
        _motor1.set(ControlMode.PercentOutput, Constants.Arm.DOWN_SPEED);
    }
    
    /**
     * Stops the arm. BRAKE mode will resist movement.
     */
    public void stop()
    {
        _usingPID = false;
        _motor1.set(ControlMode.PercentOutput, 0);
    }

    // Command factory methods for button bindings

    /**
     * Creates a command that raises the arm while active.
     * Stops arm when command ends.
     */
    public Command raiseCommand()
    {
        return run(this::raise).finallyDo(interrupted -> stop());
    }
    
    /**
     * Creates a command that lowers the arm while active.
     * Stops arm when command ends.
     */
    public Command lowerCommand()
    {
        return run(this::lower).finallyDo(interrupted -> stop());
    }
    
    /**
     * Creates a command that stops the arm immediately.
     */
    public Command stopCommand()
    {
        return runOnce(this::stop);
    }

    // Position feedback methods

    /**
     * Gets current arm angle from encoder.
     * @return Angle in degrees
     */
    public double getAngle()
    {
        return _encoder.getDistance();
    }
    
    /**
     * Resets encoder to zero. Call when arm is at known position.
     */
    public void resetEncoder()
    {
        _encoder.reset();
    }

    // PID control methods

    /**
     * Sets target angle for PID control. Arm will automatically move to and hold this angle.
     * @param angle Target angle in degrees
     */
    public void setAngle(double angle)
    {
        _targetAngle = angle;
        _usingPID = true;
        _pidController.setSetpoint(angle);
    }
    
    /**
     * Checks if arm is at target angle (within tolerance).
     * @return true if at setpoint
     */
    public boolean atSetpoint()
    {
        return _pidController.atSetpoint();
    }
    
    /**
     * Creates a command that sets target angle.
     * Command runs until arm reaches setpoint.
     * @param angle Target angle in degrees
     */
    public Command setAngleCommand(double angle)
    {
        return run(() -> setAngle(angle))
            .until(this::atSetpoint);
    }
    
    /**
     * Creates a command that moves arm to intake position.
     */
    public Command goToIntakeCommand()
    {
        return setAngleCommand(Constants.Arm.INTAKE_ANGLE);
    }
    
    /**
     * Creates a command that moves arm to stow position.
     */
    public Command goToStowCommand()
    {
        return setAngleCommand(Constants.Arm.STOW_ANGLE);
    }
}