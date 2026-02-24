// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.managersubsystems.RobotState;
import frc.robot.subsystems.shooter.ShotCalculator;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO flywheelIO;
  private final FlywheelInputsAutoLogged loggedInputs = new FlywheelInputsAutoLogged();

  public Flywheel(FlywheelIO flywheelIO) {
    this.flywheelIO = flywheelIO;
  }

  public void setFlywheelControl(ControlRequest request) {
    flywheelIO.setFlywheelOutput(request);
  }

  @Override
  public void periodic() {
    flywheelIO.updateInputs(loggedInputs);

    Logger.processInputs("Shooter/Flywheel", loggedInputs);

    RobotState.instance()
        .setFlywheelVelocity(ShotCalculator.instance().getTargetFlywheelVelocity());
  }

  public void setGains(Slot0Configs gains) {
    flywheelIO.updateFlywheelSlot0Configs(gains);
  }
}
