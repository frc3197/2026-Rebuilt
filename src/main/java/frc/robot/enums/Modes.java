package frc.robot.enums;

public class Modes {

  public static enum TurretMode {
    TRACKING_HUB,
    PASSING,
    IDLE,
    MANUAL
  }

  public static enum IntakeDeployMode {
    RETRACTING,
    DEPLOYING,
    IDLE_RETRACTED,
    IDLE_DEPLOYED,
    MANUAL
  }

  public static enum IntakeSpinMode {
    INTAKING,
    OUTTAKING,
    IDLE,
    MANUAL
  }

  public static enum FlywheelMode {
    IDLE,
    PREPARE,
    SHOOTING,
    FRENZY,
    MANUAL
  }
}
