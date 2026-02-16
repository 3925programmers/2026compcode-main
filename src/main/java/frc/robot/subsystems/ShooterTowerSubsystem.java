// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.commands.neoMotor;
import frc.robot.commands.ShooterTowerCommand;

public class ShooterTowerSubsystem extends SubsystemBase implements ShooterTowerCommand {
  public SparkMax switchMotor = new SparkMax(Constants.ShooterTowerConstants.SWITCH_MOTOR, MotorType.kBrushless);
  public SparkMax intakeMotor = new SparkMax(Constants.ShooterTowerConstants.INTAKE_MOTOR, MotorType.kBrushless);
  public SparkFlex shooterMotor = new SparkFlex(Constants.ShooterTowerConstants.SHOOTER_MOTOR, MotorType.kBrushless);
  
    public ShooterTowerSubsystem() {
      configs();
    }
  
    public void configs() {
      SparkMaxConfig switchConfig = new SparkMaxConfig();
        switchConfig.idleMode(Constants.ShooterTowerConstants.IDLE_MODE)
                    .inverted(Constants.ShooterTowerConstants.INVERTED)
                    .smartCurrentLimit(Constants.ShooterTowerConstants.CURRENT_LIMIT);

      SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(Constants.ShooterTowerConstants.IDLE_MODE)
                     .inverted(false)
                     .smartCurrentLimit(Constants.ShooterTowerConstants.CURRENT_LIMIT);

      SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(Constants.ShooterTowerConstants.IDLE_MODE)
                     .inverted(false)
                     .smartCurrentLimit(Constants.ShooterTowerConstants.CURRENT_LIMIT);
      
      intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      switchMotor.configure(switchConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  
  public void rotate(int direction) {
      switchMotor.set(0.5 * direction);
      shooterMotor.stopMotor();
      shooterMotor.setVoltage(0);
      intakeMotor.set(-0.5 * direction);
  }

  public void rotateSwitchAndShooter(int direction) {
    switchMotor.set(0.75 * direction);
    shooterMotor.set(0.5 * direction);
    intakeMotor.set(0.5 * direction);
  }

  public void rotateShooterOnly(int direction) {
    switchMotor.set(0);
    shooterMotor.set(0.5 * direction);
    intakeMotor.set(0);
  }

  public void stop() {
    switchMotor.set(0);
    shooterMotor.set(0);
    intakeMotor.set(0);
  }

  // public void toggleInverted() {
  //   Constants.NeoMotorConstants.INVERTED = !Constants.NeoMotorConstants.INVERTED;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
