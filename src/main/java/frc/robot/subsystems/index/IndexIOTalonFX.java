// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.HardwareID;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class IndexIOTalonFX extends RealSubsystem implements IndexIO {

  private final TalonFX feedMotor;
  private final TalonFXS spindexMotorController;

  public IndexIOTalonFX() {
    feedMotor = new TalonFX(IndexConstants.FEED_MOTOR_ID, HardwareID.MAIN_CANBUS);
    feedMotor.getConfigurator().apply(IndexConstants.FEED_CONFIG);

    spindexMotorController = new TalonFXS(IndexConstants.SPINDEX_MOTOR_ID, HardwareID.MAIN_CANBUS);
    spindexMotorController.getConfigurator().apply(IndexConstants.SPINDEX_CONFIG);
  }

  protected void configureHardware() {}

  @Override
  public void updateInputs(IndexInputs inputs) {}

  @Override
  public void setFeedMotorVoltage(Voltage volts) {
    feedMotor.setVoltage(volts.magnitude());
  }

  @Override
  public void setSpindexMotorVoltage(Voltage volts) {
    spindexMotorController.setVoltage(volts.magnitude());
  }
}
