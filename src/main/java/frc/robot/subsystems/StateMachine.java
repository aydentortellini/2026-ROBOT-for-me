// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.nio.file.FileSystem;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.firecontrol.ShotCalculator;
import frc.lib.team9410.PowerRobotContainer;
import frc.lib.team9410.subsystems.PositionSubsystem;
import frc.lib.team9410.subsystems.VelocitySubsystem;
import frc.lib.team9410.subsystems.VelocityTorqueSubsystem;
import frc.robot.Constants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.utils.FieldUtils.GameZone;
import frc.robot.utils.FieldUtils;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.TurretHelpers;

// Subsystem that holds high-level robot state and drives transitions.
public class StateMachine extends SubsystemBase {

  public enum RobotState {
    READY,
    SHOOTING
  }

  private RobotState wantedState = RobotState.READY;
  private RobotState currentState = RobotState.READY;
  private RobotState previousState = RobotState.READY;

  // Position subsystems
  public final PositionSubsystem shooterHood = new PositionSubsystem(Constants.Shooter.HOOD_CONFIG);
  public final PositionSubsystem intakeWrist = new PositionSubsystem(Constants.Intake.WRIST_CONFIG);

  // Velocity subsystems
  public final VelocitySubsystem shooter = new VelocitySubsystem(Constants.Shooter.FLYWHEEL_CONFIG);
  public final VelocityTorqueSubsystem intakeRoller = new VelocityTorqueSubsystem(Constants.Intake.ROLLER_CONFIG);
  public final VelocitySubsystem spindexer = new VelocitySubsystem(Constants.Spindexer.SPINDEXER_CONFIG);
  public final VelocitySubsystem feeder = new VelocitySubsystem(Constants.Feeder.FEEDER_CONFIG);

  private final Vision vision = new Vision();
  private final Dashboard dashboard = new Dashboard();

  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  //Shoot-on-the-move fire control
  private final ShotCalculator shotCalc;
  private ShotCalculator.LaunchParameters lastSOTMResult = ShotCalculator.LaunchParameters.INVALID;
  //Static aim angle (degrees) toward current target — updated each shooting cycle.
  private double staticAimAngleDeg = 0.0;

  //NT pose publishing
  private final StructPublisher<Pose2d> posePublisher =
      NetworkTableInstance.getDefault()
          .getStructTopic("Robot/Pose", Pose2d.struct)
          .publish();
  private final StructArrayPublisher<SwerveModuleState> moduleStatePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Robot/SwerveStates", SwerveModuleState.struct)
          .publish();

  private boolean gyroReset = false;

  private int intakeTimer = 0;

  private String bestLimelight = "";

  private boolean matchStarted = false;


  public StateMachine() {
    ShotCalculator.Config cfg = new ShotCalculator.Config();
    cfg.launcherOffsetX = -0.20;        // negative — shooter is behind robot center
    cfg.launcherOffsetY = 0.0;
    cfg.shooterAngleOffsetRad = Math.PI; // shooter faces rear
    cfg.maxSOTMSpeed = 3.0;
    cfg.minScoringDistance = 0.5;
    cfg.maxScoringDistance = 5.0;
    shotCalc = new ShotCalculator(cfg);
    shotCalc.loadShotLUT(TurretConstants.SHOT_LUT);
  }

  @Override
  public void periodic() {
    handleStateTransitions();
    executeState();
    PowerRobotContainer.setData("robotState", currentState.name());
    PowerRobotContainer.setData("currentZone", getZoneFromPRC());
    setRobotPose();
    // logPoseToWpilog(drivetrain.getState().Pose);
    
  
    // Map<String, Object> robotData = PowerRobotContainer.getAllData();
    // SmartDashboard.putData("robotData", builder -> {
    // builder.setSmartDashboardType("robotData");
    // for (String key : robotData.keySet()) {
    //     builder.addDoubleProperty(key, () -> robotData.get(key), v -> robotData.put(key, v));}});

  
    SmartDashboard.putString("gameZone", FieldUtils.getZone(drivetrain.getState().Pose).name());
    posePublisher.set(drivetrain.getState().Pose);
    moduleStatePublisher.set(drivetrain.getState().ModuleStates);

    // PowerRobotContainer.setData("SpindexerVelocity", spindexer.getVelocityMotor().getVelocity().getValueAsDouble());
    // PowerRobotContainer.setData("ShooterVelocity", shooter.getVelocityMotor().getVelocity().getValueAsDouble());
    // PowerRobotContainer.setData("FeederVelocity", feeder.getVelocityMotor().getVelocity().getValueAsDouble());
  }

  public void resetGyro () {
    gyroReset = false;
  }

  // private void logPoseToWpilog(Pose2d pose) {
  //   if (m_odometryLog == null) {
  //     var log = DataLogManager.getLog();
  //     m_odometryLog = StructLogEntry.create(log, "Odometry", Pose2d.struct);
  //     m_robotPositionLog = StructLogEntry.create(log, "RobotPosition", Translation2d.struct);
  //   }
  //   m_odometryLog.append(pose);
  //   m_robotPositionLog.append(pose.getTranslation());
  // }

  public void setRobotPose () {
    bestLimelight = vision.getBestLimelight();

    // Don't use vision when no Limelight has valid data (avoids bad/default pose from empty/wrong table on boot).
    if (bestLimelight == null || bestLimelight.isEmpty()) {
      // Skip vision pose update; dashboard/PRC still use current drivetrain pose below.
    } else {
      // SetRobotOrientation MUST use the gyro yaw (not vision-derived yaw) so MegaTag2 can
      // correctly resolve the 3D pose on both alliances. Call it for all cameras every loop.
      double gyroYawDeg = drivetrain.getState().Pose.getRotation().getDegrees();
      double pitchDeg   = drivetrain.getPigeon2().getPitch().getValueAsDouble();
      double rollDeg    = drivetrain.getPigeon2().getRoll().getValueAsDouble();
      LimelightHelpers.SetRobotOrientation(Constants.Vision.LEFT_TABLE,   gyroYawDeg, 0, pitchDeg, 0, rollDeg, 0);
      LimelightHelpers.SetRobotOrientation(Constants.Vision.RIGHT_TABLE,  gyroYawDeg, 0, pitchDeg, 0, rollDeg, 0);
      LimelightHelpers.SetRobotOrientation(Constants.Vision.TURRET_TABLE, gyroYawDeg, 0, pitchDeg, 0, rollDeg, 0);

      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
      if (mt2 != null && mt2.tagCount > 0) {
        Pose2d newPose = mt2.pose;
        // Reject poses outside field bounds (e.g. 0,0 or garbage from cold Limelight).
        double x = newPose.getX();
        double y = newPose.getY();
        if (x < Constants.Field.X_MIN - Constants.Field.TOL || x > Constants.Field.X_MAX + Constants.Field.TOL
            || y < Constants.Field.Y_MIN - Constants.Field.TOL || y > Constants.Field.Y_MAX + Constants.Field.TOL) {
          // Bad pose; do not reset or add to estimator.
        } else {
          // System.out.println(matchStarted);
          if (!gyroReset || !matchStarted) {
            // newPose = new Pose2d(newPose.getX(), newPose.getY(), drivetrain.getState().Pose.getRotation());
            drivetrain.resetPose(newPose);
            gyroReset = true;
          }
          // else {
          //   newPose = new Pose2d(newPose.getX(), newPose.getY(), drivetrain.getState().Pose.getRotation());
          // }
          // drivetrain.resetPose(newPose);
          drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
          drivetrain.addVisionMeasurement(newPose, Utils.fpgaToCurrentTime(mt2.timestampSeconds));
        }
      }
    }
      Translation2d translationToPoint = drivetrain.getState().Pose.getTranslation().minus(Constants.Field.HOPPER_RED);
      double linearDistance = translationToPoint.getNorm();

      // PowerRobotContainer.setData("distanceToHopper", linearDistance);
      SmartDashboard.putNumber("currentPoseX", drivetrain.getState().Pose.getX());
      SmartDashboard.putNumber("currentPoseY", drivetrain.getState().Pose.getY());
      SmartDashboard.putNumber("currentPoseYaw", drivetrain.getState().Pose.getRotation().getDegrees());
      SmartDashboard.putString("bestLimelight", bestLimelight);
  }

  private void handleStateTransitions() {
    if (currentState != wantedState) {
      previousState = currentState;
      if (canTransitionTo(wantedState)) {
        currentState = wantedState;
      }
    }
  }

  /** Override to add transition guards (e.g. require subsystems ready). */
  private boolean canTransitionTo(RobotState target) {
    return true;
  }

  private void executeState() {
    switch (currentState) {
      case READY:
        executeReady();
        break;
      case SHOOTING:
        executeShooting();
        break;
      default:
        break;
    }
  }

  private void executeReady() {
    shooter.brake();
    spindexer.brake();
    feeder.brake();
    if (intakeWrist.getSetpointRotations() > Constants.Intake.INTAKE_IDLE){
      intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
    }
    intakeTimer = 0;
  }

  public GameZone getZoneFromPRC () {
    Pose2d pose = drivetrain.getState().Pose;

    return FieldUtils.getZone(pose);
  }

  private void executeShooting() {
    intakeTimer++;
    Pose2d pose = drivetrain.getState().Pose;
    GameZone zone = FieldUtils.getZone(pose);
    boolean inOurZone = getAllianceZone() == zone;

    Translation2d target = inOurZone
        ? (zone == GameZone.BLUE_ALLIANCE ? Constants.Field.HOPPER_BLUE : Constants.Field.HOPPER_RED)
        : getTargetCornerLocation();

    // if (
    //   !(pose.getTranslation().minus(target).getNorm() > 3.5) && 
    //   !(vision.getTurretCanSeeTags())
    // ) {
    //   feeder.brake();
    //   spindexer.brake();
    //   shooter.brake();
    //   intakeRoller.brake();
    //   return;
    // }

    runShootingToTarget(target);
    if (intakeWrist.getSetpointRotations() >= Constants.Intake.INTAKE_IDLE) {
      if (intakeTimer % 50 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_FEED);
      } else if (intakeTimer % 25 == 0) {
        intakeWrist.setPositionRotations(Constants.Intake.INTAKE_IDLE);
      }
    }
  }

  // Runs shooter, hood, feeder, and spindexer toward the given target (hopper or corner).
  private void runShootingToTarget(Translation2d target) {
    Pose2d pose = drivetrain.getState().Pose;
    double distance = pose.getTranslation().minus(target).getNorm();
    SmartDashboard.putNumber("distanceToHopper", distance);

    // --- SOTM solver ---
    // Field-relative velocity: rotate robot-relative speeds by heading
    ChassisSpeeds robotVel = drivetrain.getState().Speeds;
    ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, pose.getRotation());

    // hubForward: unit vector from target toward field center so "behind hub" detection works
    Translation2d fieldCenter = new Translation2d(8.75, 4.0);
    Translation2d toCenter = fieldCenter.minus(target);
    double toCenterNorm = toCenter.getNorm();
    Translation2d hubForward = toCenterNorm > 0.01
        ? toCenter.div(toCenterNorm)
        : new Translation2d(1, 0);

    LimelightHelpers.PoseEstimate latestMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(bestLimelight);
    double visionConf = (latestMT2 != null && latestMT2.tagCount > 0) ? 1.0 : 0.5;
    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
        pose, fieldVel, robotVel, target, hubForward, visionConf);
    lastSOTMResult = shotCalc.calculate(inputs);

    // Determine shooter velocity and hood angle
    double shooterVelo;
    double hoodPos;
    if (lastSOTMResult.isValid()) {
      shooterVelo = lastSOTMResult.rpm(); // rps (LUT is in rps)
      // ShotCalculator stores hood angle in degrees; convert back to rotations for motor
      double hoodDeg = shotCalc.getHoodAngle(lastSOTMResult.solvedDistanceM());
      hoodPos = hoodDeg / 360.0;
    } else {
      shooterVelo = TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance);
      hoodPos = TurretConstants.HOOD_ANGLE_INTERPOLATOR.getInterpolatedValue(distance);
    }

    // Update static aim angle (used by SwerveDriveCommand as fallback).
    staticAimAngleDeg = Math.toDegrees(TurretHelpers.getRadiansToPoint(pose, target)) + 180.0;

    double actualShooterVelo = shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble();
    SmartDashboard.putNumber("shooterVelocity", shooterVelo);
    SmartDashboard.putNumber("shooterVelocityActual", actualShooterVelo);
    SmartDashboard.putNumber("shooterVelocityError", shooterVelo - actualShooterVelo);
    SmartDashboard.putNumber("shooterHoodPos", hoodPos);
    SmartDashboard.putNumber("shooterHoodPosActual", shooterHood.getPositionRotations());
    SmartDashboard.putNumber("rpmOffset", shotCalc.getOffset());
    SmartDashboard.putNumber("sotmConfidence", lastSOTMResult.confidence());
    SmartDashboard.putBoolean("sotmValid", lastSOTMResult.isValid());

    boolean slowSpindexer = SmartDashboard.getBoolean("slowSpindexer", false);
    boolean slowShooter   = SmartDashboard.getBoolean("slowShooter", false);
    boolean slowFeeder    = SmartDashboard.getBoolean("slowFeeder", false);

    // if (!slowShooter) {
    //   shooterVelo += 1; // old tuning hack — disabled, FireControl already sets correct RPM
    // }

    shooter.setVelocity(shooterVelo);
    shooterHood.setPositionRotations(
        MathUtil.clamp(hoodPos, ShooterConstants.SHOOTER_HOOD_MIN, ShooterConstants.SHOOTER_HOOD_MAX));

    // Heading gate: only feed/spin when robot is aimed within tolerance
    double targetHeadingRad = lastSOTMResult.isValid()
        ? lastSOTMResult.driveAngle().getRadians()
        : Math.toRadians(staticAimAngleDeg);
    double headingErrorDeg = Math.abs(Math.toDegrees(
        MathUtil.angleModulus(pose.getRotation().getRadians() - targetHeadingRad)));
    boolean headingOK = headingErrorDeg < 5.0; // degrees — tune this
    SmartDashboard.putNumber("headingErrorDeg", headingErrorDeg);
    SmartDashboard.putBoolean("headingOK", headingOK);

    // double velocityThreshold = TurretConstants.SHOOTER_VELOCITY_INTERPOLATOR.getInterpolatedValue(distance) - 1.5; // old — fought FireControl
    double velocityThreshold = shooterVelo - 1.5;
    boolean shooterAtSpeed = shooter.getVelocityMotor().getRotorVelocity().getValueAsDouble() > velocityThreshold;

    if (headingOK && shooterAtSpeed) {
      feeder.setVelocity(slowFeeder ? -60 : -2 * shooterVelo);
      spindexer.setVelocity(slowSpindexer ? 70 : 80);
    } else {
      feeder.brake();
      spindexer.brake();
    }
  }

  // Latest SOTM fire-control result. Updated every shooting cycle. */
  public ShotCalculator.LaunchParameters getLastSOTMResult() {
    return lastSOTMResult;
  }

  // The ShotCalculator instance (used by simulation to query hood angles etc.). */
  public ShotCalculator getShotCalc() {
    return shotCalc;
  }

  /** Static aim angle (degrees, field frame) toward the current target. Used as heading fallback. */
  public double getStaticAimAngleDeg() {
    return staticAimAngleDeg;
  }

  public void setWantedState(RobotState state) {
    wantedState = state;
  }

  public RobotState getCurrentState() {
    return currentState;
  }

  public RobotState getWantedState() {
    return wantedState;
  }

  public RobotState getPreviousState() {
    return previousState;
  }

  /** Convenience: is the robot on the blue alliance? */
  public boolean isBlueAlliance() {
    if (DriverStation.getAlliance().isEmpty())
      return true;
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }

  /** Distance (m) inward from each side when targeting a corner (field: blue right = 0,0, red left = max, max). */
  private static final double CORNER_TARGET_OFFSET_M = 1.0;

  /**
   * Returns a target position 1 m inward from each side that forms the corner (end wall and side wall).
   * Blue right = (0,0), red left = (max, max). Picks the nearest of our alliance's two corners.
   */
  public Translation2d getTargetCornerLocation() {
    Translation2d robotPos = drivetrain.getState().Pose.getTranslation();
    if (isBlueAlliance()) {
      // Blue corners: 1m from blue wall (low X) and 1m from bottom (low Y) or top (high Y)
      Translation2d fromBottom = new Translation2d(
          Constants.Field.BLUE_BOTTOM_CORNER.getX() + CORNER_TARGET_OFFSET_M,
          Constants.Field.BLUE_BOTTOM_CORNER.getY() + CORNER_TARGET_OFFSET_M);
      Translation2d fromTop = new Translation2d(
          Constants.Field.BLUE_TOP_CORNER.getX() + CORNER_TARGET_OFFSET_M,
          Constants.Field.BLUE_TOP_CORNER.getY() - CORNER_TARGET_OFFSET_M);
      return robotPos.getY() > 4 ? fromTop : fromBottom;
    } else {
      // Red corners: 1m from red wall (high X) and 1m from bottom (low Y) or top (high Y)
      Translation2d fromBottom = new Translation2d(
          Constants.Field.RED_BOTTOM_CORNER.getX() - CORNER_TARGET_OFFSET_M,
          Constants.Field.RED_BOTTOM_CORNER.getY() + CORNER_TARGET_OFFSET_M);
      Translation2d fromTop = new Translation2d(
          Constants.Field.RED_TOP_CORNER.getX() - CORNER_TARGET_OFFSET_M,
          Constants.Field.RED_TOP_CORNER.getY() - CORNER_TARGET_OFFSET_M);
      // System.out.println(robotPos.getY() > 4 ? fromTop : fromBottom);
      return robotPos.getY() > 4 ? fromTop : fromBottom;
    }
  }

  public GameZone getAllianceZone() {
    if (isBlueAlliance()) {
      return GameZone.BLUE_ALLIANCE;
    } else {
      return GameZone.RED_ALLIANCE;
    }
  }


  // thanks wpilib
  // https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    // If we have no alliance, we cannot be enabled, therefore no hub.
    if (alliance.isEmpty()) {
      return false;
    }
    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in teleop.
    if (gameData.isEmpty()) {
      return true;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        return true;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active = switch (alliance.get()) {
      case Red -> !redInactiveFirst;
      case Blue -> redInactiveFirst;
    };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      return true;
    } else if (matchTime > 105) {
      // Shift 1
      return shift1Active;
    } else if (matchTime > 80) {
      // Shift 2
      return !shift1Active;
    } else if (matchTime > 55) {
      // Shift 3
      return shift1Active;
    } else if (matchTime > 30) {
      // Shift 4
      return !shift1Active;
    } else {
      // End game, hub always active.
      return true;
    }
  }

  public void setMatchStarted () {
    matchStarted = true;
  }
}
