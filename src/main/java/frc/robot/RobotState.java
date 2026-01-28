package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.util.VirtualSubsystem;
import frc.robot.util.RobotMode.ShooterMode;

import org.littletonrobotics.junction.Logger;

public class RobotState extends VirtualSubsystem {
  private static RobotState instance;

  private Distance climberHeight = Inches.of(0);

  private ShooterMode shooterMode = ShooterMode.IDLE;

  private final Mechanism2d primaryMechanism2d;

  private final MechanismRoot2d primaryMechanismRoot;
  private final MechanismLigament2d climberLigament2d;

  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d = new MechanismLigament2d("RobotBase", 150, 0, 24,
      new Color8Bit(Color.kBlue));

  private final String key;

  private RobotState(String key) {

    this.key = key;

    primaryMechanism2d = new Mechanism2d(500, 300);
    climberLigament2d = new MechanismLigament2d("ClimberLigament", climberHeight.in(Centimeters), 90);

    robotBaseRoot = primaryMechanism2d.getRoot("2dBaseRoot", 225, 20);
    robotBaseRoot.append(baseLigament2d);

    primaryMechanismRoot = primaryMechanism2d.getRoot("2dPrimary", 300, 20);
    primaryMechanismRoot.append(climberLigament2d);

    SmartDashboard.putData("Mech2d", primaryMechanism2d);

  }

  public static RobotState instance() {
    if (instance == null) {
      instance = new RobotState("measured");
    }
    return instance;
  }

  @Override
  public void periodic() {
    visualize();
  }

  public Distance getClimberHeight() {
    return climberHeight;
  }

  public void setClimberHeight(Distance climberHeight) {
    this.climberHeight = climberHeight;
  }

  public ShooterMode getShooterMode() {
    return shooterMode;
  }

  public void setShooterMode(ShooterMode shooterMode) {
    this.shooterMode = shooterMode;
  }

  private void visualize() {
    Pose3d climberPose = new Pose3d(CLIMBER_ATTACH_OFFSET.getTranslation(), CLIMBER_ATTACH_OFFSET.getRotation())
        .transformBy(
            new Transform3d(
                new Translation3d(
                    0, 0, -this.climberHeight.in(Meters)),
                new Rotation3d()));

    climberLigament2d.setLength(climberHeight.in(Centimeters) + 103.5);

    Logger.recordOutput("RobotState/Climber/" + key, climberPose);

    Logger.recordOutput("RobotState/ShooterMode", shooterMode);
  }

  private static final Transform3d CLIMBER_ATTACH_OFFSET = new Transform3d(
      new Translation3d(Inches.of(2.125), Inches.of(-11.5), Inches.of(3.5)),
      new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(90)));
}
