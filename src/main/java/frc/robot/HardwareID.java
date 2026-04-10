package frc.robot;

import com.ctre.phoenix6.CANBus;

public class HardwareID {

  public static final CANBus DRIVETRAIN_CANBUS = new CANBus("Drivetrain");
  public static final CANBus MAIN_CANBUS = CANBus.roboRIO();

  public interface ShooterHardwareID {
    public static final int FLYWHEEL_LEFT_MOTOR_ID = 16;
    public static final int FLYWHEEL_RIGHT_MOTOR_ID = 17;
    public static final int TURRET_ROTATION_ID = 11;
  }

  public interface IndexHardwareID {
    public static final int FEED_MOTOR_ID = 10;
    public static final int SPINDEX_MOTOR_ID = 9;
  }

  public interface IntakeHardwareID {
    public static final int DEPLOY_MOTOR_ID = 13;
    public static final int DEPLOY_CANCODER_ID = 17;

    public static final int LEFT_SPIN_MOTOR_ID = 15;
    public static final int RIGHT_SPIN_MOTOR_ID = 14;
  }

  public interface ClimberHardwareID {
    public static final int climberMotor = 27;
  }
}
