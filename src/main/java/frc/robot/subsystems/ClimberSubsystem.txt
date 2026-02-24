// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ClimberCommand;

public class ClimberSubsystem extends SubsystemBase implements ClimberCommand {
  public SparkMax rightClimb = new SparkMax(Constants.ClimberConstants.RIGHT_CLIMB_MOTOR, MotorType.kBrushless);
  public SparkMax leftClimb = new SparkMax(Constants.ClimberConstants.LEFT_CLIMB_MOTOR, MotorType.kBrushless);


  public ClimberSubsystem() {
    configs();
  }

  public void configs() {
    SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
      rightMotorConfig.idleMode(Constants.ClimberConstants.IDLE_MODE)
                      .inverted(Constants.ClimberConstants.INVERTED)
                      .smartCurrentLimit(Constants.ClimberConstants.CURRENT_LIMIT);

    SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
      leftMotorConfig.idleMode(Constants.ClimberConstants.IDLE_MODE)
                     .smartCurrentLimit(Constants.ClimberConstants.CURRENT_LIMIT)
                     .follow(Constants.ClimberConstants.RIGHT_CLIMB_MOTOR);
    leftClimb.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightClimb.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void rotate(int direction) {
    rightClimb.set(0.9 * direction);
  }

  public void stop() {
    rightClimb.set(0);
  }
}
