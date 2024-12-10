// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.stateSpace.StateSpaceController;

public class Pivot extends SubsystemBase {
  
  private TalonFX m_pivotMotor = new TalonFX(PivotConstants.k_motorPort);
  private CANcoder m_cancoder = new CANcoder(PivotConstants.k_cancoderPort);

  private StateSpaceController<N2, N1, N2> m_controller;

  /** Creates a new Pivot. */
  public Pivot() {
    configEncoder();
    Vector<N2> initialState = getOutput();
    m_controller = new StateSpaceController<N2, N1, N2>(PivotConstants.k_config, this::getOutput, this::applyInput, initialState);
  }

  public void configEncoder() {
    m_cancoder.clearStickyFaults();
    m_cancoder.getConfigurator().apply(new CANcoderConfiguration(), 0.05);

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withAbsoluteSensorRange(PivotConstants.k_absoluteSensorRange)
            .withSensorDirection(PivotConstants.k_cancoderInvert)
            .withMagnetOffset(PivotConstants.k_encoderOffset));

    m_cancoder.getConfigurator().apply(canCoderConfiguration, 0.2);
  }

  private Vector<N2> getOutput() {
    double position = m_cancoder.getAbsolutePosition().getValueAsDouble(); // position is in mechanism rotations
    double velocity = m_cancoder.getVelocity().getValueAsDouble();
    return VecBuilder.fill(position, velocity);
  }

  private void applyInput(Vector<N1> inputs) {
    double volts = inputs.get(0);
    m_pivotMotor.setVoltage(volts);
  }

  public void setPosition(double pivotPosition) {
    m_controller.setReference(VecBuilder.fill(pivotPosition, 0.0));
  }
}
