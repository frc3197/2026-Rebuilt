package frc.robot;

import com.ctre.phoenix6.CANBus;

public class HardwareID {

  public static final CANBus DRIVETRAIN_CANBUS = new CANBus("drivetrain");
  public static final CANBus MAIN_CANBUS = CANBus.roboRIO();

  public interface ShooterHardwareID {

    // Motors
    public static final int FLYWHEEL_MOTOR_ID = 12;
    public static final int TURRET_ROTATION_ID = 11;
  }

  public interface IndexHardwareID {
    // Motors
    public static final int FEED_MOTOR_ID = 10;
    public static final int INDEX_MOTOR_ID = 9;
  }

  public interface IntakeHardwareID {
    public static final int DEPLOY_MOTOR_ID = -1;
    public static final int DEPLOY_CANCODER_ID = -1;

    public static final int SPIN_MOTOR_ID = -1;
    public static final int SPIN_CANCODER_ID = -1;
  }

  public interface ClimberHardwareID {
    public static final int leftClimberMotorID = -1;
    public static final int rightClimberMotorID = -1;
  }
}
