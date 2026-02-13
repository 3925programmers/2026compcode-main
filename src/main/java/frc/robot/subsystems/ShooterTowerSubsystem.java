// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
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
  public SparkMax mainMotor = new SparkMax(Constants.NeoMotorConstants.SWITCH_MOTOR, MotorType.kBrushless);
  
    public ShooterTowerSubsystem() {
      configs();
    }
  
    public void configs() {
      SparkMaxConfig rotateConfig = new SparkMaxConfig();
        rotateConfig.idleMode(Constants.NeoMotorConstants.IDLE_MODE)
                    .inverted(Constants.NeoMotorConstants.INVERTED)
                    .smartCurrentLimit(Constants.NeoMotorConstants.CURRENT_LIMIT);
  
      mainMotor.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  
    public void rotate(int direction) {
      int invertedSign = Constants.NeoMotorConstants.INVERTED ? -1 : 1;
      mainMotor.set(0.75 * direction * invertedSign);

  }

  public void stop() {
    mainMotor.set(0);
  }

  // public void toggleInverted() {
  //   Constants.NeoMotorConstants.INVERTED = !Constants.NeoMotorConstants.INVERTED;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
