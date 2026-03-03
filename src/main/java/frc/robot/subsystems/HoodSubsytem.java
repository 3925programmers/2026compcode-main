// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsytem extends SubsystemBase {

    private final Servo hoodServo = new Servo(0);

    public void setPosition(double position) {
        position = MathUtil.clamp(position, 0.0, 1.0);
        hoodServo.set(position);
    }
}

