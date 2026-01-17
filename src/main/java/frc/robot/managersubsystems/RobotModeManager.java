package frc.robot.managersubsystems;

import frc.robot.util.VirtualSubsystem;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class RobotModeManager extends VirtualSubsystem {

  private static String robotMode = "Hello";

  public static RobotModeManager instance;

  public RobotModeManager() {}

  public static RobotModeManager instance() {
    if (instance == null) {
      instance = new RobotModeManager();
    }
    return instance;
  }

  public static void setRobotMode(String mode) {
    robotMode = mode;
  }

  @Override
  public void periodic() {
    visualize();
  }

  public void visualize() {
    Logger.recordOutput("Robot Mode", robotMode);
  }
}