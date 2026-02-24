// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/** Add your docs here. */
public class QuestIOReal implements QuestIO {

  QuestNav questNav = new QuestNav();

  Pose3d questPose = new Pose3d();

  public QuestIOReal() {}

  protected void configureHardware() {}

  @Override
  public void updateInputs(QuestInputs inputs) {
    inputs.isConnected = questNav.isConnected();
    inputs.questPose = questPose;
  }

  @Override
  public void questPeriodic() {
    questNav.commandPeriodic();

    PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

    if (poseFrames.length > 0) {
      Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();
      this.questPose = questPose;

      // Transform by the mount pose to get your robot pose
      Pose3d robotPose = questPose.transformBy(QuestConstants.ROBOT_TO_QUEST.inverse());
    }
  }

  public void acceptVisionPose(Pose2d pose) {
    Pose2d newQuestPose =
        pose.transformBy(
            new Transform2d(
                QuestConstants.ROBOT_TO_QUEST.getX(),
                QuestConstants.ROBOT_TO_QUEST.getY(),
                QuestConstants.ROBOT_TO_QUEST.getRotation().toRotation2d()));
    questNav.setPose(new Pose3d(newQuestPose));
  }
}
