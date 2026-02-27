// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  public final Limelight limelight = new Limelight(getName(), 0);
  public CommandSwerveDrivetrain drivetrain;
  public CommandXboxController joystick;
  public SwerveRequest.FieldCentric drive;
  public double MaxSpeed;
  public double MaxAngularRate;
   // public final boolean tx = limelight.getTX();
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoAlign(CommandSwerveDrivetrain drivetrain, 
                      CommandXboxController joystick, 
                      SwerveRequest.FieldCentric drive,
                      double MaxSpeed,
                      double MaxAngularRate) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
        this.drivetrain = drivetrain;
        this.joystick = joystick;
        this.drive = drive;
        this.MaxSpeed = MaxSpeed;
        this.MaxAngularRate = MaxAngularRate;
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tx = limelight.getTX();

    // Simple P control for turning
    double kP = 0.02;  // tune this
    double rotation = tx * kP * MaxAngularRate;
    // if (Math.abs(tx) < 1.0) {
    //   rotation = 0;
    // }

    drivetrain.setControl(
      drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
           .withVelocityY(-joystick.getLeftX() * MaxSpeed)
           .withRotationalRate(rotation)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
