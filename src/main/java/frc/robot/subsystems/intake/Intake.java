// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Controls intake wheels for grabbing/ejecting game pieces.
 * Hardware: 1x VictorSPX motor controller.
 */
public class Intake extends SubsystemBase
{
    /**
     * Possible states for intake wheels.
     */
    public enum IntakeState
    {
        IN,   // Pulling game pieces in
        OUT,  // Pushing game pieces out
        OFF   // Stopped
    }

    private final WPI_VictorSPX _motor;
    private IntakeState _currentState = IntakeState.OFF;

    /**
     * Creates a new Intake subsystem.
     */
    public Intake()
    {
        _motor = new WPI_VictorSPX(Constants.CAN.INTAKE_MOTOR);
        _motor.configFactoryDefault();
        
        // COAST mode - wheels spin freely when not powered (better for intakes)
        _motor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets intake to a specific state.
     * @param state Desired state (IN, OUT, or OFF)
     */
    public void set(IntakeState state)
    {
        _currentState = state;
        
        double speed = switch (state)
        {
            case IN -> Constants.Intake.IN_SPEED;
            case OUT -> Constants.Intake.OUT_SPEED;
            case OFF -> 0.0;
        };
        
        _motor.set(ControlMode.PercentOutput, speed);
    }
    
    /**
     * Runs wheels inward to grab game pieces.
     * Call continuously while button is held.
     */
    public void runIn()
    {
        set(IntakeState.IN);
    }
    
    /**
     * Runs wheels outward to eject game pieces.
     * Call continuously while button is held.
     */
    public void runOut()
    {
        set(IntakeState.OUT);
    }
    
    /**
     * Stops the wheels.
     */
    public void stop()
    {
        set(IntakeState.OFF);
    }

    // Status methods

    public boolean isIntaking()
    {
        return _currentState == IntakeState.IN;
    }
    
    public boolean isOuttaking()
    {
        return _currentState == IntakeState.OUT;
    }
    
    public boolean isStopped()
    {
        return _currentState == IntakeState.OFF;
    }
    
    public IntakeState getState()
    {
        return _currentState;
    }

    // Command factory methods

    /**
     * Creates a command that runs intake inward while active.
     * Stops when command ends.
     */
    public Command intakeCommand()
    {
        return run(this::runIn).finallyDo(interrupted -> stop());
    }
    
    /**
     * Creates a command that runs intake outward while active.
     * Stops when command ends.
     */
    public Command outtakeCommand()
    {
        return run(this::runOut).finallyDo(interrupted -> stop());
    }
    
    /**
     * Creates a command that stops intake immediately.
     */
    public Command stopCommand()
    {
        return runOnce(this::stop);
    }
    
    /**
     * Creates a command that sets intake to a specific state.
     * Useful in command compositions.
     * @param state Desired state
     */
    public Command setStateCommand(IntakeState state)
    {
        return runOnce(() -> set(state));
    }
}