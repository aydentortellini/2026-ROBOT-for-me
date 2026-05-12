// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * MapleSim intake zone gated by a state-machine boolean.
 *
 * <p>Front-of-robot in-frame intake, 0.6&nbsp;m wide, capacity 1 fuel ball. Tick
 * {@link #update()} from {@code Robot.simulationPeriodic()}; when the supplier is true,
 * the intake zone is extended and collects fuel the robot drives over.
 * {@link #getHeldCount()} is the sim source of truth for "has fuel", and
 * {@link #consume()} removes the held piece when a shot is spawned.
 */
public class IntakeSim {
  private final IntakeSimulation intakeSim;
  private final BooleanSupplier intakeActive;

  public IntakeSim(SwerveDriveSimulation driveSim, BooleanSupplier intakeActive) {
    this.intakeActive = intakeActive;
    this.intakeSim =
        IntakeSimulation.InTheFrameIntake(
            "Fuel", driveSim, Meters.of(0.6), IntakeSide.FRONT, 1);
  }

  public void update() {
    if (intakeActive.getAsBoolean()) {
      intakeSim.startIntake();
    } else {
      intakeSim.stopIntake();
    }
  }

  public int getHeldCount() {
    return intakeSim.getGamePiecesAmount();
  }

  /** Removes one held piece. Returns true if a piece was consumed. */
  public boolean consume() {
    return intakeSim.obtainGamePieceFromIntake();
  }
}
