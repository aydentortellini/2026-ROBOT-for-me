// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.firecontrol.FuelPhysicsSim;
import frc.robot.subsystems.StateMachine.RobotState;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Full-field ball physics sim. Only created in simulation. */
  private FuelPhysicsSim ballSim = null;
  private int simBallCooldown = 0;

  public Robot() {
    SignalLogger.setPath("/U/ctre-logs/");
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // DataLogManager.start();
    // DataLogManager.logNetworkTables(false);
    SmartDashboard.putBoolean("driveInverted",false);
    SmartDashboard.putBoolean("velocityLock", false);

    SmartDashboard.putBoolean("slowSpindexer", false);
    SmartDashboard.putBoolean("slowShooter", false);
    SmartDashboard.putBoolean("slowFeeder", false);
  }

  @Override
  public void robotPeriodic() {
    SignalLogger.writeDouble("Voltage", RobotController.getBatteryVoltage(), "V");
    SignalLogger.writeDouble("CAN Utilization", RobotController.getCANStatus().percentBusUtilization, "%");
    SignalLogger.writeDouble("CAN Error Count", RobotController.getCANStatus().receiveErrorCount, "#");
    // m_robotContainer.getStateMachine().setRobotPose();
    m_robotContainer.setAuto();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    
    m_robotContainer.getStateMachine().setMatchStarted();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
    m_robotContainer.getStateMachine().setMatchStarted();
    m_robotContainer.resetState();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void simulationInit() {
    var sm = m_robotContainer.getStateMachine();
    
    ballSim = new FuelPhysicsSim("Sim/Fuel");
    ballSim.enable();
    ballSim.placeFieldBalls();

    // Configure robot shape for bumper collision and intake detection.
    // Adjust width/length to match your robot's frame perimeter from the CAD model.
    ballSim.configureRobot(
        0.70, // robot width along Y axis (m)
        0.70, // robot length along X axis (m)
        0.10, // bumper height (m)
        () -> sm.drivetrain.getState().Pose,
        () -> ChassisSpeeds.fromRobotRelativeSpeeds(
            sm.drivetrain.getState().Speeds,
            sm.drivetrain.getState().Pose.getRotation()));
  }

  @Override
  public void simulationPeriodic() {
    if (ballSim == null) return;
    ballSim.tick();

    var sm = m_robotContainer.getStateMachine();
    if (sm.getCurrentState() != RobotState.SHOOTING) {
      simBallCooldown = 0;
      return;
    }

    var result = sm.getLastSOTMResult();
    if (!result.isValid() || simBallCooldown > 0) {
      if (simBallCooldown > 0) simBallCooldown--;
      return;
    }

    // Compute launch vector from the SOTM result
    var pose    = sm.drivetrain.getState().Pose;
    // Shooter faces rear (Math.PI offset matches shooterAngleOffsetRad in ShotCalculator.Config)
    double heading = pose.getRotation().getRadians() + Math.PI;
    // Exit speed: rps * wheel circumference * slip factor (0.127m wheel, 0.7 slip — tune to robot)
    double exitSpeedMps = result.rpm() * Math.PI * 0.127 * 0.7;
    // Hood angle from ShotCalculator (degrees) drives the vertical component
    double hoodAngleDeg = sm.getShotCalc().getHoodAngle(result.solvedDistanceM());
    double hoodRad = Math.toRadians(hoodAngleDeg > 0 ? hoodAngleDeg : 16.2); // fallback ~0.045 rot
    double horzSpeed = exitSpeedMps * Math.cos(hoodRad);
    double vertSpeed = exitSpeedMps * Math.sin(hoodRad);

    Translation3d launchPos = new Translation3d(
        pose.getX() - 0.20 * Math.cos(pose.getRotation().getRadians()), // rear offset
        pose.getY() - 0.20 * Math.sin(pose.getRotation().getRadians()),
        0.80); // approximate shooter height (m)
    Translation3d launchVel = new Translation3d(
        horzSpeed * Math.cos(heading),
        horzSpeed * Math.sin(heading),
        vertSpeed);

    ballSim.launchBall(launchPos, launchVel, result.rpm() * 60.0); // convert rps→RPM for spin
    simBallCooldown = 25; // ~0.5 s between spawned balls
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
