// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IndexIOSim implements IndexIO {

  private final DCMotor feedGearbox = DCMotor.getKrakenX60(1);

  private final DCMotorSim feedMotorSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(feedGearbox, 1, 0.7), feedGearbox);

  public IndexIOSim() {}

  @Override
  public void updateInputs(IndexInputs inputs) {
    inputs.indexMotorSuppliedVoltage = feedMotorSim.getInputVoltage();
  }

  @Override
  public void setFeedMotorVoltage(Voltage volts) {
    feedMotorSim.setInputVoltage(volts.magnitude());
  }
}
