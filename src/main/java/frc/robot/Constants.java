// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Robot-wide constants and configuration values.
 * Organized into nested classes for clarity.
 */
public final class Constants
{
    private Constants()
    {
    }

    /**
     * CAN bus device IDs.
     * IDs must be unique (0-62) and match Phoenix Tuner configuration.
     */
    public static class CAN
    {
        public static final int ARM_MOTOR_1 = 1;
        public static final int ARM_MOTOR_2 = 2;
        public static final int INTAKE_MOTOR = 3;
    }

    /**
     * RoboRIO Digital I/O port assignments.
     * Quadrature encoders require two ports (A and B channels).
     */
    public static class DIO
    {
        public static final int ARM_ENCODER_A = 0;
        public static final int ARM_ENCODER_B = 1;
    }

    /**
     * Controller configuration constants.
     */
    public static class Controls
    {
        // Minimum input value to register as movement
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    /**
     * Arm subsystem constants.
     */
    public static class Arm
    {
        // Manual control speeds
        public static final double UP_SPEED = 0.5;
        public static final double DOWN_SPEED = -0.3;
        
        // Motor inversion settings
        public static final boolean MOTOR_2_INVERTED = false; // TODO: Set to true if motor 2 is inverted

        // Encoder conversion factor
        // Formula: 360 degrees / pulses per revolution
        public static final double ENCODER_DISTANCE_PER_PULSE = 360.0 / 256.0;
        
        // Preset positions in degrees
        public static final double INTAKE_ANGLE = 45.0;
        public static final double STOW_ANGLE = 0.0;

        // PID Controller gains
        public static final double kP = 0.02;
        public static final double kI = 0.0;
        public static final double kD = 0.001;
        public static final double kG = 0.1; // Gravity feedforward
        
        // Position tolerance in degrees
        public static final double POSITION_TOLERANCE = 2.0;
    }

    /**
     * Intake subsystem constants.
     */
    public static class Intake
    {
        // Positive speed pulls game pieces in, negative pushes out
        public static final double IN_SPEED = 0.7;
        public static final double OUT_SPEED = -0.5;
    }

    /**
     * General robot constants.
     */
    public static class General
    {
        public static final double MOTOR_VOLTAGE = 12.0;
    }

    /**
     * Driver station port assignments.
     */
    public static class OperatorConstants
    {
        public static final int kDriverControllerPort = 0;
    }
}
