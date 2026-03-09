// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managersubsystems.RobotState;
import org.littletonrobotics.junction.Logger;

public class Index extends SubsystemBase {

  private final IndexIO indexIO;
  private final IndexInputsAutoLogged inputs = new IndexInputsAutoLogged();

  public Index(IndexIO indexIO) {
    this.indexIO = indexIO;
  }

  @Override
  public void periodic() {
    indexIO.updateInputs(inputs);

    Logger.processInputs("Index", inputs);

    RobotState.instance().setIndexCurrentDraw(inputs.feedDrawAmps.plus(inputs.spindexDrawAmps));
  }

  public Command setFeedMotorCommand(Voltage volts) {
    return Commands.runOnce(() -> indexIO.setFeedMotorVoltage(volts), this);
  }

  public Command setSpindexMotorCommand(Voltage volts) {
    return Commands.runOnce(() -> indexIO.setSpindexMotorVoltage(volts), this);
  }

  public void setFeedMotor(Voltage volts) {
    indexIO.setFeedMotorVoltage(volts);
  }

  public void setSpindexMotor(Voltage volts) {
    indexIO.setSpindexMotorVoltage(volts);
  }

  public void setGains(Slot0Configs gains) {
    indexIO.setGains(gains);
  }

  public void setFeedRequest(ControlRequest request) {
    indexIO.setFeedMotorRequest(request);
  }
}
