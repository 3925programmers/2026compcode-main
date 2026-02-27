// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Limelight;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class AutoAlign extends Command {
  public final RobotContainer robotContainer = new RobotContainer();
  public final Limelight limelight = new Limelight(getName(), 0);
  public final CommandSwerveDrivetrain drivetrain = robotContainer.drivetrain;
  public final CommandXboxController joystick = robotContainer.joystick;
  public double tx = limelight.getTX();
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
              .withDeadband(robotContainer.MaxSpeed * 0.1).withRotationalDeadband(robotContainer.MaxAngularRate * 0.1) // Add a 10% deadband
              .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
   // public final boolean tx = limelight.getTX();
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoAlign() {}
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * robotContainer.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * robotContainer.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(tx) // Drive counterclockwise with negative X (left)
        )
      );
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      tx = limelight.getTX();
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
            drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * robotContainer.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * robotContainer.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * robotContainer.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
