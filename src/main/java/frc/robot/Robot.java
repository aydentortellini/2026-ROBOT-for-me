// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

// import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.sim.IntakeSim;
import frc.robot.subsystems.StateMachine.RobotState;
import org.ironmaple.simulation.SimulatedArena;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /** Sim-only intake zone. Tracks held fuel and gates shot spawning. */
  private IntakeSim intakeSim = null;

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
    // Populate the Arena2026Rebuilt field with fuel balls and scoring targets.
    SimulatedArena.getInstance().resetFieldForAuto();

    var sm = m_robotContainer.getStateMachine();
    intakeSim = new IntakeSim(
        sm.drivetrain.getDriveSimulation(),
        () -> sm.getCurrentState() == RobotState.INTAKING);
  }

  @Override
  public void simulationPeriodic() {
    var sm = m_robotContainer.getStateMachine();

    // Toggle the intake zone before the arena ticks so contact detection runs with
    // the correct extended/retracted state for this frame.
    if (intakeSim != null) intakeSim.update();

    // Advance MapleSim physics. Inside this call, each module's SimulatedMotorController
    // pushes encoder positions into the TalonFX/CANcoder sim states so CTRE's odometry
    // sees fresh sensor values.
    SimulatedArena.getInstance().simulationPeriodic();

    // Push MapleSim's gyro reading into the Pigeon2 sim state.
    sm.drivetrain.updateSimState();
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
