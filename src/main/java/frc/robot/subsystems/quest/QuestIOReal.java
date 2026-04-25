// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.LoggingConstants;
import frc.robot.managersubsystems.RobotState;
import frc.robot.util.LoggedTunableNumber;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/** Add your docs here. */
public class QuestIOReal implements QuestIO {

  QuestNav questNav = new QuestNav();

  private Pose3d questPose = new Pose3d();
  private Pose3d robotPose = new Pose3d();
  private double timestamp = 0.0;

  private LoggedTunableNumber R_TO_Q_X =
      new LoggedTunableNumber(
          "Quest X", Units.metersToInches(-QuestConstants.ROBOT_TO_QUEST.getY()));
  private LoggedTunableNumber R_TO_Q_Y =
      new LoggedTunableNumber(
          "Quest Y", Units.metersToInches(QuestConstants.ROBOT_TO_QUEST.getX()));

  public QuestIOReal() {}

  @Override
  public void updateInputs(QuestInputs inputs) {
    inputs.isConnected = questNav.isConnected();
    inputs.questPose = questPose;
    inputs.robotPose = robotPose;
  }

  @Override
  public void questPeriodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    for (PoseFrame questFrame : poseFrames) {
      // Make sure the Quest was tracking the pose for this frame
      if (questFrame.isTracking()) {
        // Get the pose of the Quest
        questPose = questFrame.questPose3d();
        // Get timestamp for when the data was sent
        timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        if (!LoggingConstants.tuningMode) {
          robotPose = questPose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
        } else {
          robotPose =
              questPose.transformBy(
                  new Transform3d(
                          Units.inchesToMeters(R_TO_Q_Y.getAsDouble()),
                          Units.inchesToMeters(-R_TO_Q_X.getAsDouble()),
                          QuestConstants.ROBOT_TO_QUEST.getZ(),
                          QuestConstants.ROBOT_TO_QUEST.getRotation())
                      .inverse());
        }
      }
    }
  }

  public void acceptVisionPose(Pose2d pose) {
    System.out.println("Accepting quest pose: " + pose.getX() + ", " + pose.getY());
    Pose2d poseWithGyro =
        new Pose2d(pose.getX(), pose.getY(), RobotState.instance().getRobotPose().getRotation());

    Pose2d newQuestPose;
    if (!LoggingConstants.tuningMode) {
      newQuestPose =
          poseWithGyro.transformBy(
              new Transform2d(
                  QuestConstants.ROBOT_TO_QUEST.getX(),
                  QuestConstants.ROBOT_TO_QUEST.getY(),
                  QuestConstants.ROBOT_TO_QUEST.getRotation().toRotation2d()));
    } else {
      newQuestPose =
          poseWithGyro.transformBy(
              new Transform2d(
                  Units.inchesToMeters(R_TO_Q_Y.getAsDouble()),
                  Units.inchesToMeters(-R_TO_Q_X.getAsDouble()),
                  QuestConstants.ROBOT_TO_QUEST.getRotation().toRotation2d()));
    }
    questNav.setPose(new Pose3d(newQuestPose));
  }

  @Override
  public void setRawQuestPosition(Pose3d pose) {
    questNav.setPose(pose);
  }

  @Override
  public boolean isConnected() {
    return questNav.isConnected();
  }

  @Override
  public Pose2d getRobotPosition() {
    return robotPose.toPose2d();
  }

  @Override
  public double getTimestamp() {
    return timestamp;
  }

  @Override
  public void setRobotPosition(Pose2d pose) {
    Pose2d newQuestPose =
        pose.transformBy(
            new Transform2d(
                QuestConstants.ROBOT_TO_QUEST.getX(),
                QuestConstants.ROBOT_TO_QUEST.getY(),
                QuestConstants.ROBOT_TO_QUEST.getRotation().toRotation2d()));
    questNav.setPose(new Pose3d(newQuestPose));
  }

  @Override
  public Pose3d getRawQuestPose() {
    return questPose;
  }
}
