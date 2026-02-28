// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.quest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import org.littletonrobotics.junction.Logger;

public class Quest extends SubsystemBase {

  private final QuestIO questIO;
  private final QuestInputsAutoLogged inputs = new QuestInputsAutoLogged();
  private final VisionConsumer driveVisionConsumer;

  public Quest(QuestIO questIO, VisionConsumer driveVisionConsumer) {
    this.questIO = questIO;
    this.driveVisionConsumer = driveVisionConsumer;
  }

  @Override
  public void periodic() {
    questIO.questPeriodic();

    questIO.updateInputs(inputs);

    Logger.processInputs("Quest", inputs);

    driveVisionConsumer.accept(
        questIO.getRobotPosition(),
        questIO.getTimestamp(),
        VecBuilder.fill(0.0, 0.0, Double.POSITIVE_INFINITY));
  }

  public void acceptVisionPose(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    questIO.acceptVisionPose(visionRobotPoseMeters);
  }

  public boolean isQuestConnected() {
    return inputs.isConnected;
  }
}
