// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** An example command that uses an example subsystem. */
public interface ShooterTowerCommand extends Subsystem {
  void rotate(int direction);
  void stop();
  // void toggleInverted();
}
