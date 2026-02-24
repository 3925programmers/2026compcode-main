// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ShooterTowerConstants {
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final boolean INVERTED = false;
    public static final int CURRENT_LIMIT = 60;
    public static final int ACTUATOR_PWM_PORT = 9;
    public static final double ACTUATOR_PID_INCREMENT = 0.05;
    public static final double ACTUATOR_MIN_POS = 0.0;
    public static final double ACTUATOR_MAX_POS = 1.0;
    public static final double ACTUATOR_START_POS = 0.5;
    public static final int INTAKE_MOTOR = 1;
    public static final int SHOOTER_MOTOR = 2;
    public static final int SWITCH_MOTOR = 3;
  }

  public static class ClimberConstants {
    public static final int RIGHT_CLIMB_MOTOR = 5;
    public static final int LEFT_CLIMB_MOTOR = 4;
    public static final int CURRENT_LIMIT = 60;
    public static final boolean INVERTED = false;
    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
  }
}
