// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;

  private ShooterInputsAutoLogged loggedShooter = new ShooterInputsAutoLogged();

  public Shooter(ShooterIO shooterIO) {

    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(loggedShooter);
  }
}
