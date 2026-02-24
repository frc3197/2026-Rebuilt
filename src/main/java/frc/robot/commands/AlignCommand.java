// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Align;
import frc.robot.subsystems.drive.Drive;

// Align command that drives robot to the target pose
public class AlignCommand extends Command {

  // Subsystems
  private final Align align;
  private final Drive drive;

  // Target to drive to
  private final Pose2d targetPose;

  public AlignCommand(Align align, Drive drive, Pose2d targetPose) {

    this.align = align;
    this.drive = drive;
    this.targetPose = targetPose;

    addRequirements(align, drive);
  }

  @Override
  public void execute() {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            align.alignWithTarget(targetPose, drive.getPose()), drive.getRotation()));
  }
}
