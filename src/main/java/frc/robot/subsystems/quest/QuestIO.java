// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface QuestIO {
  @AutoLog
  public static class QuestInputs {
    public Pose3d questPose = new Pose3d();
    public Pose3d robotPose = new Pose3d();
    public boolean isConnected = false;
  }

  public default void updateInputs(QuestInputs inputs) {}

  public default void questPeriodic() {}

  public default Pose2d getRobotPosition() {
    return new Pose2d();
  }

  public default boolean isConnected() {
    return false;
  }

  public default double getTimestamp() {
    return 0.0;
  }

  public default void acceptVisionPose(Pose2d pose) {}
}
