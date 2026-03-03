// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.HoodSubsytem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  private final HoodSubsytem hoodSubsytem;

  private final InterpolatingDoubleTreeMap hoodMap =
    new InterpolatingDoubleTreeMap();

    // Replace with real hub coordinates
  private static final Pose2d HUB_POSE =
    new Pose2d(4.449, 4.035, new Rotation2d());

  private final Limelight limelight;
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController joystick;
  private final SwerveRequest.FieldCentric drive;
  private final double MaxSpeed;
  private final double MaxAngularRate;

  private final PIDController turnPID = new PIDController(0.02, 0, 0.001);

  public AutoAlign(CommandSwerveDrivetrain drivetrain,
                   Limelight limelight,
                   CommandXboxController joystick,
                   SwerveRequest.FieldCentric drive,
                   double MaxSpeed,
                   double MaxAngularRate) {

    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.joystick = joystick;
    this.drive = drive;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;

    turnPID.setTolerance(1.0);   // degrees of tx
    turnPID.setSetpoint(0.0);    // we want tx = 0 (centered)

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {

    double rotation = 0;
    SmartDashboard.putString("hello", "i work");

    if (limelight.hasTargets()) {

      double tx = limelight.getTX();

      rotation = turnPID.calculate(-tx);
      SmartDashboard.putNumber("AutoAlign Rotation Output", rotation);

      // Clamp output so we don't spin violently
      rotation = MathUtil.clamp(rotation, 
                                -MaxAngularRate, 
                                MaxAngularRate);

      // Optional: stop tiny oscillations
      if (turnPID.atSetpoint()) {
        rotation = 0;
      }
    }

    drivetrain.setControl(
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
             .withVelocityY(-joystick.getLeftX() * MaxSpeed)
             .withRotationalRate(rotation)
    );
  }

  @Override
  public void end(boolean interrupted) {
    turnPID.reset();
  }

  @Override
  public boolean isFinished() {
    return false; // runs while button held
  }
}