package frc.robot.enums;

public class Modes {

  public static enum TurretMode {
    TRACKING_HUB,
    PASSING,
    IDLE,
    MANUAL,
  }

  public static enum HoodMode {
    TRACKING,
    DOWN
  }

  public static enum IntakeDeployMode {
    RETRACTING,
    DEPLOYING,
    IDLE_RETRACTED,
    IDLE_DEPLOYED,
    FLOPPING,
  }

  public static enum IntakeSpinMode {
    INTAKING,
    OUTTAKING,
    IDLE,
  }

  public static enum FlywheelMode {
    IDLE,
    PREPARE,
    SHOOTING,
    FRENZY
  }

  public static enum ClimbCameraMode {
    CLIMB_RED,
    CLIMB_BLUE,
    APRIL_TAGS
  }
}
