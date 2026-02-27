// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.ShooterTowerCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterTowerSubsystem;

public class RobotContainer {
    private static final int DELAYED_SPIN_MOTOR_ID = 6;
    private static final double DELAYED_SPIN_MOTOR_SPEED = -0.25;
    private static final double DELAYED_SPIN_SECONDS = 1.0;

    public double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.65).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // private final Telemetry logger = new Telemetry(MaxSpeed);


    public final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AutoAlign autoAlign = new AutoAlign(drivetrain, joystick, drive, MaxSpeed, MaxAngularRate);


    //subsystem setup
    public final ShooterTowerSubsystem shooterTowerSubsystem = new ShooterTowerSubsystem();
    private final SparkMax delayedSpinMotor = new SparkMax(DELAYED_SPIN_MOTOR_ID, MotorType.kBrushless);

    private boolean reverseMotor = false;

    public RobotContainer() {
        configureBindings();
    }
    
    private Command spinShooter() {
        return Commands.runEnd(
            () -> {
                shooterTowerSubsystem.rotate(reverseMotor ? -1 : 1);
            },
        this::stopShooter,
        shooterTowerSubsystem
        );
    }



    private Command spinSwitchAndShooterReverse() {
        Command shooterCommand = Commands.sequence(
            Commands.runEnd(
                () -> {
                    shooterTowerSubsystem.rotateShooterOnly(-1);
                },
                this::stopShooter,
                shooterTowerSubsystem
            ).withTimeout(1.5),
            Commands.runEnd(
                () -> {
                    shooterTowerSubsystem.rotateSwitchAndShooter(-1);
                },
                this::stopShooter,
                shooterTowerSubsystem
            )
        );

        Command delayedMotorStart = Commands.sequence(
            Commands.waitSeconds(DELAYED_SPIN_SECONDS),
            Commands.runOnce(() -> delayedSpinMotor.set(DELAYED_SPIN_MOTOR_SPEED))
        );

        return Commands.parallel(shooterCommand, delayedMotorStart);
    }

    private void stopShooter() {
        shooterTowerSubsystem.stop();
        delayedSpinMotor.set(0);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().toggleOnTrue(autoAlign);
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> delayedSpinMotor.set(0)).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Actuator: LB steps down one PID point, RB steps up one PID point.
        joystick.leftBumper().onTrue(Commands.runOnce(shooterTowerSubsystem::decrementActuatorPidPoint));
        joystick.rightBumper().onTrue(Commands.runOnce(shooterTowerSubsystem::incrementActuatorPidPoint));
        // Reset the field-centric heading on left stick press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().toggleOnTrue(spinShooter());
        joystick.y().toggleOnTrue(spinSwitchAndShooterReverse());
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
