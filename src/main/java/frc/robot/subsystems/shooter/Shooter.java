// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO shooterIO;

  private ShooterInputsAutoLogged loggedShooter = new ShooterInputsAutoLogged();
  private final Supplier<Pose3d> robotPoseSupplier;

  public Shooter(ShooterIO shooterIO, Supplier<Pose3d> robotPoseSupplier) {

    this.robotPoseSupplier = robotPoseSupplier;
    this.shooterIO = shooterIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(loggedShooter);

    Logger.recordOutput(
        "Shooter/Turret Location",
        robotPoseSupplier.get().plus(ShooterConstants.ROBOT_TO_TURRET_CENTER));
  }
}
