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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.commands.neoMotor;
import frc.robot.commands.ShooterTowerCommand;

public class ShooterTowerSubsystem extends SubsystemBase implements ShooterTowerCommand {
  public SparkMax switchMotor = new SparkMax(Constants.ShooterTowerConstants.SWITCH_MOTOR, MotorType.kBrushless);
  public SparkMax intakeMotor = new SparkMax(Constants.ShooterTowerConstants.INTAKE_MOTOR, MotorType.kBrushless);
  public SparkFlex shooterMotor = new SparkFlex(Constants.ShooterTowerConstants.SHOOTER_MOTOR, MotorType.kBrushless);
  public Servo actuatorMotor = new Servo(Constants.ShooterTowerConstants.ACTUATOR_PWM_PORT);
  private double actuatorPosition = Constants.ShooterTowerConstants.ACTUATOR_START_POS;
  private int actuatorDirection = 0;
  
    public ShooterTowerSubsystem() {
      configs();
      actuatorMotor.set(actuatorPosition);
    }
  
    public void configs() {
      SparkMaxConfig switchConfig = new SparkMaxConfig();
        switchConfig.idleMode(Constants.ShooterTowerConstants.IDLE_MODE)
                    .inverted(Constants.ShooterTowerConstants.INVERTED)
                    .smartCurrentLimit(Constants.ShooterTowerConstants.CURRENT_LIMIT);

      SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(Constants.ShooterTowerConstants.IDLE_MODE)
                     .inverted(true)
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
    switchMotor.set(0.9 * direction);
    shooterMotor.set(0.65 * direction);
    intakeMotor.set(0.5 * direction);
  }

  public void rotateShooterOnly(int direction) {
    switchMotor.set(0);
    shooterMotor.set(0.5 * direction);
    intakeMotor.set(0);
  }

  public void moveActuator(int direction) {
    actuatorDirection = Integer.signum(direction);
  }

  public void stopActuator() {
    actuatorDirection = 0;
    actuatorMotor.set(actuatorPosition);
  }

  public void stop() {
    switchMotor.set(0);
    shooterMotor.set(0);
    intakeMotor.set(0);
    stopActuator();
  }

  // public void toggleInverted() {
  //   Constants.NeoMotorConstants.INVERTED = !Constants.NeoMotorConstants.INVERTED;
  // }

  @Override
  public void periodic() {
    if (actuatorDirection != 0) {
      actuatorPosition = MathUtil.clamp(
          actuatorPosition + (Constants.ShooterTowerConstants.ACTUATOR_STEP_PER_LOOP * actuatorDirection),
          Constants.ShooterTowerConstants.ACTUATOR_MIN_POS,
          Constants.ShooterTowerConstants.ACTUATOR_MAX_POS);
    }
    actuatorMotor.set(actuatorPosition);
  }
}
