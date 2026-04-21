// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToPointCommand extends Command {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public double MAX_DRIVE_TO_POINT_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // * 0.75;
  public double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.5)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE = RotationsPerSecond.of(1)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double STATIC_FRICTION_CONSTANT = 0.085; // Adjust this value based on your robot's characteristics
  public double SKEW_COMPENSATION = 0. - 0.03; // Adjust this value based on your robot's characteristics

  private final Swerve drivetrain;
  private Translation2d targetPoint;
  private double rotationTolerance;
  private Rotation2d targetRotationToPoint;

  /** Creates a new DriveCommand. */
  public TurnToPointCommand(
      Swerve drivetrain,
      Translation2d point,
      double tolerance) {
    this.drivetrain = drivetrain;
    this.targetPoint = point;
    this.rotationTolerance = tolerance;

    Translation2d targetPoint = isBlueAlliance() ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED;
    // Get the robot's current position on the field
    Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();

    // Find the vector from the robot to the target
    double deltaX = targetPoint.getX() - robotPosition.getX();
    double deltaY = targetPoint.getY() - robotPosition.getY();
    // double deltaDistance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    // Find the angle from the robot to the target in field coordinates
    double targetAngleFieldRelative = Math.atan2(deltaY, deltaX);

    double targetAngleRobotRelative = !isBlueAlliance()
        ? Rotation2d.fromRadians(targetAngleFieldRelative).getDegrees()
        : Rotation2d.fromRadians(targetAngleFieldRelative).rotateBy(Rotation2d.fromDegrees(180)).getDegrees();

    this.targetRotationToPoint = Rotation2d.fromDegrees(targetAngleRobotRelative);

    addRequirements(drivetrain);
  }

  public boolean isBlueAlliance() {
    if (DriverStation.getAlliance().isEmpty())
      return true;
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(
        0.0,
        0.0,
        targetRotationToPoint.getDegrees(),
        Swerve.DriveMode.ROTATION_LOCK);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math
        .abs(drivetrain.getState().Pose.getRotation().minus(targetRotationToPoint).getDegrees()) < rotationTolerance;
  }
}