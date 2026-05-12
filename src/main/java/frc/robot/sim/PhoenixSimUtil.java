// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Sim-state bridge adapters between MapleSim and Phoenix 6.
 *
 * <p>Each adapter implements {@link SimulatedMotorController}. On every MapleSim
 * physics sub-tick (~5 per 20ms), MapleSim calls {@link #updateControlSignal} with
 * the simulated encoder + mechanism state. The adapter pushes that state into the
 * Phoenix sim state so CTRE's odometry reads correct positions/velocities, and
 * returns the voltage CTRE wants applied so MapleSim's motor physics can advance.
 *
 * <p>Pattern adapted from the Shenzhen-Robotics-Alliance AKit-MapleSim-Enhanced
 * reference.
 */
public final class PhoenixSimUtil {
  private PhoenixSimUtil() {}

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private final TalonFXSimState simState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.simState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      simState.setRawRotorPosition(encoderAngle);
      simState.setRotorVelocity(encoderVelocity);
      simState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return simState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXWithRemoteCANcoderSim extends TalonFXMotorControllerSim {
    private final CANcoderSimState cancoderSimState;

    public TalonFXWithRemoteCANcoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.cancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      cancoderSimState.setRawPosition(mechanismAngle);
      cancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }
}
