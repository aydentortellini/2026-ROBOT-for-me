// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.DriveUtil;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.FieldUtils.GameZone;
import frc.robot.utils.TurretHelpers;
import frc.lib.team9410.PowerRobotContainer;
import frc.robot.Constants;
import frc.robot.constants.LocationConstants;
import frc.robot.constants.OIConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Swerve.DriveMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveDriveCommand extends Command {
  public double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  public double MAX_DRIVE_TO_POINT_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // * 0.75;
  public double MAX_ANGULAR_RATE = RotationsPerSecond.of(1.5)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double MAX_DRIVE_TO_POINT_ANGULAR_RATE = RotationsPerSecond.of(1)
      .in(RadiansPerSecond); // 0.75 rotations per second in radians per second unit
  public double STATIC_FRICTION_CONSTANT = 0.085; // Adjust this value based on your robot's characteristics
  public double SKEW_COMPENSATION = 0. - 0.03; // Adjust this value based on your robot's characteristics

  private final Swerve drivetrain;
  private final CommandXboxController controller;
  private final boolean autoDrive;
  private final PIDController driveToPointController;
  private Pose2d requestedPose;
  private double poseTolerance;
  /**
   * If set, scales MAX_DRIVE_TO_POINT_SPEED for drive-to-point (0.0 to 1.0 = 0%
   * to 100%).
   */
  private final Double driveToPointSpeedMultiplier;
  private final boolean disableRotationLock;
  /** If set, used for SOTM heading lock during shooting. */
  private final StateMachine stateMachine;

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = null;
    this.poseTolerance = -1;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  /**
   * Teleop drive with SOTM heading lock during shooting.
   * Pass the StateMachine so the command can read the SOTM aim angle.
   */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      StateMachine stateMachine) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = null;
    this.poseTolerance = -1;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = stateMachine;

    addRequirements(drivetrain);
  }

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = -1.0;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  /** Creates a new DriveCommand. */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointSpeedMultiplier = null;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  /**
   * Creates a new DriveCommand with requested pose, pose tolerance, and
   * drive-to-point speed as a percent of default.
   *
   * @param driveToPointSpeedMultiplier scale for max drive-to-point speed (0.0 to
   *                                    1.0 = 0% to 100% of
   *                                    {@link #MAX_DRIVE_TO_POINT_SPEED})
   */
  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance,
      double driveToPointSpeedMultiplier) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointSpeedMultiplier = driveToPointSpeedMultiplier;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = false;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  public SwerveDriveCommand(
      Swerve drivetrain,
      CommandXboxController controller,
      boolean autoDrive,
      Pose2d requestedPose,
      double poseTolerance,
      double driveToPointSpeedMultiplier,
      boolean disableRotationLock) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    this.autoDrive = autoDrive;
    this.requestedPose = requestedPose;
    this.poseTolerance = poseTolerance;
    this.driveToPointSpeedMultiplier = driveToPointSpeedMultiplier;
    this.driveToPointController = new PIDController(3.2, 0, 0.2);
    this.disableRotationLock = disableRotationLock;
    this.stateMachine = null;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final Pose2d currentPose = drivetrain.getState().Pose;
    final double speedCoefficient = FieldUtils.getZone(currentPose) == GameZone.INTERCHANGE
        ? 1.0 //? OIConstants.INTERCHANGE_SPEED_COEFFICIENT
        : 1.0;
    Pose2d targetPose = new Pose2d();
    targetPose = requestedPose;
    ChassisSpeeds currentChassisSpeeds = drivetrain.getState().Speeds;
    double currentSpeed = Math.sqrt(
        Math.pow(currentChassisSpeeds.vxMetersPerSecond, 2) + Math.pow(currentChassisSpeeds.vyMetersPerSecond, 2));

    if (controller.rightTrigger(0.5).getAsBoolean() && stateMachine != null
        && stateMachine.getCurrentState() == RobotState.SHOOTING) {
      boolean isInverted = SmartDashboard.getBoolean("driveInverted", false);
      double inversionMultiplier = isInverted ? -1.0 : 1.0;
      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(
          controller, drivetrain, MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      double xSpeed = speeds.vxMetersPerSecond * OIConstants.MAX_SPEED_COEFFICIENT * inversionMultiplier;
      double ySpeed = speeds.vyMetersPerSecond * OIConstants.MAX_SPEED_COEFFICIENT * inversionMultiplier;

      var sotmResult = stateMachine.getLastSOTMResult();
      boolean usingSOTM = sotmResult.isValid();
      double aimDeg = usingSOTM
          ? sotmResult.driveAngle().getDegrees()
          : stateMachine.getStaticAimAngleDeg();
      double opPerspDeg = isBlueAlliance() ? 0.0 : 180.0;
      double adjustedAimDeg = aimDeg - opPerspDeg;
      SmartDashboard.putNumber("aimDeg", aimDeg);
      SmartDashboard.putNumber("adjustedAimDeg", adjustedAimDeg);
      SmartDashboard.putBoolean("usingSOTM", usingSOTM);
      SmartDashboard.putNumber("robotHeadingDeg", drivetrain.getState().Pose.getRotation().getDegrees());
      drivetrain.drive(xSpeed, ySpeed, adjustedAimDeg, DriveMode.ROTATION_LOCK);
    } else if (controller.rightTrigger(0.5).getAsBoolean()) {
      drivetrain.drive(0.0, 0.0, 0.0, DriveMode.BRAKE);
    } else if (currentPose != null && targetPose != null && (autoDrive || requestedPose != null)) {
      boolean isBlueAlliance = true;
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          isBlueAlliance = false;
        }
      }
      final double directionMultiplier = isBlueAlliance ? -1.0 : 1.0;

      final Translation2d translationToPoint = currentPose.getTranslation().minus(targetPose.getTranslation());
      final double linearDistance = translationToPoint.getNorm();
      double ff = 0;
      if (linearDistance >= Units.inchesToMeters(3)) {
        ff = STATIC_FRICTION_CONSTANT * MAX_SPEED;
      }

      double multiplier = (driveToPointSpeedMultiplier != null)
          ? Math.max(0.0, Math.min(1.0, driveToPointSpeedMultiplier))
          : 1.0;
      double maxSpeed = MAX_DRIVE_TO_POINT_SPEED * multiplier * speedCoefficient;

      final Rotation2d directionOfTravel = translationToPoint.getAngle();
      final double velocity = Math.min(Math.abs(driveToPointController.calculate(linearDistance, 0)) + ff, maxSpeed);
      final double xSpeed = velocity * directionOfTravel.getCos() * directionMultiplier;
      final double ySpeed = velocity * directionOfTravel.getSin() * directionMultiplier;

      drivetrain.drive(
          xSpeed,
          ySpeed,
          disableRotationLock ? 0.0 : targetPose.getRotation().getDegrees(),
          disableRotationLock ? Swerve.DriveMode.FIELD_RELATIVE : Swerve.DriveMode.DRIVE_TO_POINT);
    } else {

      final ChassisSpeeds speeds = DriveUtil.calculateSpeedsBasedOnJoystickInputs(controller, drivetrain,
          MAX_ANGULAR_RATE, SKEW_COMPENSATION);
      final double coeff = speedCoefficient == 1.0 ? OIConstants.MAX_SPEED_COEFFICIENT : speedCoefficient;
      double xSpeed = speeds.vxMetersPerSecond * coeff;
      double ySpeed = speeds.vyMetersPerSecond * coeff;

      // if (controller.rightTrigger(0.5).getAsBoolean()){
      // final double DRIVE_AND_SHOOT_SPEED = 0.0;
      // xSpeed = xSpeed * DRIVE_AND_SHOOT_SPEED;
      // ySpeed = ySpeed * DRIVE_AND_SHOOT_SPEED;
      // } else
      if (controller.leftTrigger(0.5).getAsBoolean()) {
        final double DRIVE_AND_INTAKE_SPEED = 0.4;
        xSpeed = xSpeed * DRIVE_AND_INTAKE_SPEED;
        ySpeed = ySpeed * DRIVE_AND_INTAKE_SPEED;
      }

      boolean isInverted = SmartDashboard.getBoolean("driveInverted", false);
      double inversionMultiplier = isInverted ? -1.0 : 1.0;
      // System.out.println("isInverted: "+inversionMultiplier);

      if (controller.a().getAsBoolean()) {
        // TEMP: A alone = SOTM aim (no trigger required)
        var sotmResult = stateMachine != null ? stateMachine.getLastSOTMResult() : null;
        boolean usingSOTM = sotmResult != null && sotmResult.isValid();
        double aimDeg;
        if (usingSOTM) {
          aimDeg = sotmResult.driveAngle().getDegrees();
        } else {
          // Fallback: fresh static aim from current pose toward hopper
          Translation2d target = isBlueAlliance() ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED;
          aimDeg = Math.toDegrees(TurretHelpers.getRadiansToPoint(drivetrain.getState().Pose, target)) + 180.0;
        }
        double opPerspDeg = isBlueAlliance() ? 0.0 : 180.0;
        double adjustedAimDeg = aimDeg - opPerspDeg;
        SmartDashboard.putNumber("aimDeg", aimDeg);
        SmartDashboard.putBoolean("usingSOTM", usingSOTM);
        drivetrain.drive(
            xSpeed * inversionMultiplier,
            ySpeed * inversionMultiplier,
            adjustedAimDeg,
            Swerve.DriveMode.ROTATION_LOCK);

        // -- old static-point A button code (kept for reference) --
        // Translation2d targetPoint = isBlueAlliance() ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED;
        // Translation2d robotPosition = drivetrain.getState().Pose.getTranslation();
        // double deltaX = targetPoint.getX() - robotPosition.getX();
        // double deltaY = targetPoint.getY() - robotPosition.getY();
        // double targetAngleFieldRelative = Math.atan2(deltaY, deltaX);
        // double targetAngleRobotRelative = Rotation2d.fromRadians(targetAngleFieldRelative).rotateBy(Rotation2d.fromDegrees(180)).getDegrees();
        // double opPerspDeg = isBlueAlliance() ? 0.0 : 180.0;
        // double adjustedTargetAngle = targetAngleRobotRelative - opPerspDeg;
        // drivetrain.drive(xSpeed * inversionMultiplier, ySpeed * inversionMultiplier, adjustedTargetAngle, Swerve.DriveMode.ROTATION_LOCK);

      } else {
        drivetrain.drive(
            xSpeed * inversionMultiplier,
            ySpeed * inversionMultiplier,
            -speeds.omegaRadiansPerSecond,
            Swerve.DriveMode.FIELD_RELATIVE);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (requestedPose != null) {
      return getIsInPosition(
          drivetrain.getState().Pose, requestedPose, drivetrain.getState().Speeds);
    }
    return false;
  }

  private double getFieldRelativeTargetRotation(double degrees) {
    Alliance alliance = DriverStation.getAlliance().get();

    if (alliance == Alliance.Red) {
      switch ((int) degrees) {
        case -90:
          return 90.0;
        case 135:
          return -45.0;
        case 90:
          return -90;
        case -135:
          return 45.0;
        default:
          return degrees;
      }
    } else {
      return degrees;
    }
  }

  private boolean getIsInPosition(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds speeds) {
    final Translation2d translationToPoint = currentPose.getTranslation().minus(targetPose.getTranslation());
    final double linearDistance = translationToPoint.getNorm();
    final double linearRotation = currentPose.getRotation().getDegrees()
        - getFieldRelativeTargetRotation(targetPose.getRotation().getDegrees());
    final double currentVelocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    final double tolerance = poseTolerance > 0 ? poseTolerance : 1;
    return linearDistance < Units.inchesToMeters(tolerance) && (Math.abs(linearRotation) < 10 || disableRotationLock); // &&
                                                                                                                       // currentVelocity
                                                                                                                       // <
                                                                                                                       // 0.01;
                                                                                                                       // //
                                                                                                                       // meters
  }

  //
  //
  // Game Specific Code
  //
  //
  ///////////////////////////////////////////////////////////

  private double getRotation(double speed, StrafeSide side, GameZone zone, Pose2d pose) {
    Alliance alliance = DriverStation.getAlliance().get();
    double centerLine = getCenterLine(zone);

    switch (side) {
      case FRONT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getY() > centerLine)
              || (alliance == Alliance.Red && pose.getY() < centerLine)) {
            return -45.0;
          } else {
            return 45.0;
          }
        } else if (speed > 0) {
          return 45.0;
        } else {
          return -45.0;
        }
      case LEFT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getX() > 2 || (alliance == Alliance.Red && pose.getX() < 2))) {
            return 135.0;
          } else {
            return 45.0;
          }
        } else if (speed > 0) {
          return 45.0;
        } else {
          return 135.0;
        }
      case RIGHT:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getX() > 2) || (alliance == Alliance.Red && pose.getX() < 2)) {
            return -135.0;
          } else {
            return -45.0;
          }
        } else if (speed > 0.0) {
          return -45.0;
        } else {
          return -135.0;
        }
      case BACK:
        if (Math.abs(speed) < 0.1) {
          if ((alliance == Alliance.Blue && pose.getY() > centerLine)
              || (alliance == Alliance.Red && pose.getY() < centerLine)) {
            return 135.0;
          } else {
            return -135.0;
          }
        } else if (speed > 0) {
          return 135.0;
        } else {
          return -135.0;
        }
      default:
        return 0.0;
    }
  }

  private double getCenterLine(GameZone zone) {
    switch (zone) {
      case BLUE_ALLIANCE:
        return LocationConstants.BLUE_ZONE_X_MID;
      case RED_ALLIANCE:
        return LocationConstants.RED_ZONE_X_MID;
      case NEUTRAL:
        return LocationConstants.NEUTRAL_X_MID;
      default:
        return -1;
    }
  }

  private double getXInput(StrafeAxis axis, StrafeSide side) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().get();
    double strafeLine;

    if ((alliance == Alliance.Blue && side == StrafeSide.LEFT)
        ||
        (alliance == Alliance.Red && side == StrafeSide.RIGHT)) {
      strafeLine = LocationConstants.FAR_WALL_Y;
    } else {
      strafeLine = LocationConstants.NEAR_WALL_Y;
    }

    final Pose2d targetPose = new Pose2d(pose.getX(), strafeLine, pose.getRotation());
    final double directionMultiplier = alliance == Alliance.Blue ? -1.0 : 1.0;

    Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
        pose, targetPose, directionMultiplier, driveToPointController, poseTolerance);

    return velocity.getX();
  }

  private double getYInput(StrafeAxis axis, StrafeSide side) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().get();
    double strafeLine;

    switch (zone) {
      case BLUE_ALLIANCE:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.BLUE_HUB_WALL_X;
        } else {
          strafeLine = LocationConstants.BLUE_ALLIANCE_WALL_X;
        }

        break;
      case RED_ALLIANCE:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.RED_ALLIANCE_WALL_X;
        } else {
          strafeLine = LocationConstants.RED_HUB_WALL_X;
        }

      case NEUTRAL:
        if ((alliance == Alliance.Blue && side == StrafeSide.FRONT)
            ||
            (alliance == Alliance.Red && side == StrafeSide.BACK)) {
          strafeLine = LocationConstants.RED_CENTER_WALL_X;
        } else {
          strafeLine = LocationConstants.BLUE_CENTER_WALL_X;
        }

      default:
        strafeLine = 0.0;
        break;
    }

    final Pose2d targetPose = new Pose2d(pose.getX(), strafeLine, pose.getRotation());
    final double directionMultiplier = alliance == Alliance.Blue ? -1.0 : 1.0;

    Translation2d velocity = DriveUtil.calculateDriveToPointVelocity(
        pose, targetPose, directionMultiplier, driveToPointController, poseTolerance);

    return velocity.getY();
  }

  /**
   * Returns the strafe side (wall) closest to the robot's current pose.
   * Uses alliance and zone to pick the correct wall X/Y constants.
   */
  private static StrafeSide getClosestSide(Swerve drivetrain) {
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    double frontWallX;
    double backWallX;
    switch (zone) {
      case BLUE_ALLIANCE:
        frontWallX = LocationConstants.BLUE_HUB_WALL_X;
        backWallX = LocationConstants.BLUE_ALLIANCE_WALL_X;
        break;
      case RED_ALLIANCE:
        frontWallX = LocationConstants.RED_HUB_WALL_X;
        backWallX = LocationConstants.RED_ALLIANCE_WALL_X;
        break;
      case NEUTRAL:
        frontWallX = alliance == Alliance.Blue ? LocationConstants.RED_CENTER_WALL_X
            : LocationConstants.BLUE_CENTER_WALL_X;
        backWallX = alliance == Alliance.Blue ? LocationConstants.BLUE_CENTER_WALL_X
            : LocationConstants.RED_CENTER_WALL_X;
        break;
      default:
        return StrafeSide.FRONT;
    }

    // Far/near wall Y are fixed field positions; LEFT/RIGHT map by alliance (see
    // getXInput).
    double distFront = Math.abs(pose.getX() - frontWallX);
    double distBack = Math.abs(pose.getX() - backWallX);
    double distToFarWall = Math.abs(pose.getY() - LocationConstants.FAR_WALL_Y);
    double distToNearWall = Math.abs(pose.getY() - LocationConstants.NEAR_WALL_Y);

    double min = Math.min(Math.min(distFront, distBack), Math.min(distToFarWall, distToNearWall));
    if (min == distFront)
      return StrafeSide.FRONT;
    if (min == distBack)
      return StrafeSide.BACK;
    if (min == distToFarWall)
      return alliance == Alliance.Blue ? StrafeSide.LEFT : StrafeSide.RIGHT;
    return alliance == Alliance.Blue ? StrafeSide.RIGHT : StrafeSide.LEFT;
  }

  private StrafeAxis getStrafeAxis(StrafeSide side) {
    if (side == StrafeSide.LEFT || side == StrafeSide.RIGHT) {
      return StrafeAxis.X;
    } else {
      return StrafeAxis.Y;
    }
  }

  private static enum StrafeAxis {
    X,
    Y
  }

  public static enum StrafeSide {
    LEFT,
    RIGHT,
    FRONT,
    BACK
  }

  /**
   * Distance (m) inward from each side when targeting a corner (field: blue right
   * = 0,0, red left = max, max).
   */
  private static final double CORNER_TARGET_OFFSET_M = 1.0;

  /** Convenience: is the robot on the blue alliance? */
  public boolean isBlueAlliance() {
    if (DriverStation.getAlliance().isEmpty())
      return true;
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }
}