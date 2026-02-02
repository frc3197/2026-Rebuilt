// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RealSubsystem;

/** Add your docs here. */
public class IndexIOTalonFX extends RealSubsystem implements IndexIO {

  private final TalonFX indexMotor;

  public IndexIOTalonFX() {
    indexMotor = new TalonFX(IndexConstants.INDEX_MOTOR_ID);
  }

  protected void configureHardware() {}

  @Override
  public void updateInputs(IndexInputs inputs) {}

  @Override
  public void setIndexMotor(double speed) {
    indexMotor.set(speed);
  }
}
