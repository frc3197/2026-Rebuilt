package frc.robot.managersubsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.IntakeMode;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.FieldConstants;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class RobotState extends VirtualSubsystem {

  // Robot Modes
  private IntakeMode intakeMode = IntakeMode.IDLE_RETRACTED;
  private FlywheelMode flywheelMode = FlywheelMode.IDLE;
  private TurretMode turretMode = TurretMode.IDLE;

  private static RobotState instance;

  private final String key;

  // Climber
  private MutDistance climberHeight = Inches.of(0).mutableCopy();
  private final MechanismLigament2d climberLigament2d;

  // Intake
  private boolean intakeFullyReatracted = false;
  private boolean intakeFullyExtended = false;

  // Turret
  private MutAngle turretRotationAngle = Degrees.of(0.0).mutableCopy();

  // Main mechanism
  private final Mechanism2d primaryMechanism2d;
  private final MechanismRoot2d primaryMechanismRoot;

  // Robot base
  private final MechanismRoot2d robotBaseRoot;
  private final MechanismLigament2d baseLigament2d =
      new MechanismLigament2d("RobotBase", 150, 0, 24, new Color8Bit(Color.kBlue));

  // Driving data
  private Pose2d robotFieldPose = Pose2d.kZero;
  private ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
  private ChassisSpeeds robotAcceleration = new ChassisSpeeds(0, 0, 0);

  private RobotState(String key) {
    this.key = key;

    primaryMechanism2d = new Mechanism2d(500, 300);
    climberLigament2d =
        new MechanismLigament2d("ClimberLigament", climberHeight.in(Centimeters), 90);

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

  // Robot pose getters & setters ----------------------------------------------

  public Pose2d getRobotPose() {
    return robotFieldPose;
  }

  public Pose3d getRobotPose3d() {
    return new Pose3d(robotFieldPose);
  }

  public void setRobotPose(Pose2d pose) {
    robotFieldPose = pose;
  }

  public ChassisSpeeds getRobotVelocity() {
    return robotVelocity;
  }

  public void setRobotVelocity(ChassisSpeeds speeds) {
    robotVelocity = speeds;
  }

  public ChassisSpeeds getRobotAcceleration() {
    return robotAcceleration;
  }

  public void setRobotAccelerations(ChassisSpeeds accelerations) {
    robotAcceleration = accelerations;
  }

  // Robot mode getters & setters -----------------------------------------------

  public IntakeMode getIntakeMode() {
    return intakeMode;
  }

  public void setIntakeMode(IntakeMode intakeMode) {
    this.intakeMode = intakeMode;
  }

  public FlywheelMode getFlywheelMode() {
    return flywheelMode;
  }

  public void setFlywheelMode(FlywheelMode flywheelMode) {
    this.flywheelMode = flywheelMode;
  }

  public TurretMode getTurretMode() {
    return turretMode;
  }

  public void setTurretMode(TurretMode turretMode) {
    this.turretMode = turretMode;
  }

  // Subsystems are below
  // Climber getters & setters ------------------------------------------------

  public Distance getClimberHeight() {
    return climberHeight;
  }

  public void setClimberHeight(Distance climberHeight) {
    this.climberHeight.mut_replace(climberHeight);
  }

  // Turret getters & setters --------------------------------------------------
  public Angle getTurretRotationAngle() {
    return turretRotationAngle;
  }

  public void setTurretRotationAngle(Angle angle) {
    turretRotationAngle.mut_replace(angle);
  }

  // Trigger helpers
  public boolean inAllianceZone() {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    return isRed
        ? robotFieldPose.getX() > FieldConstants.Red.MIN_HUB_TRACKING_X
        : robotFieldPose.getX() < FieldConstants.Blue.MAX_HUB_TRACKING_X;
  }

  public boolean inNeutralZone() {
    return robotFieldPose.getX() < FieldConstants.Red.MIN_HUB_TRACKING_X
        && robotFieldPose.getX() > FieldConstants.Blue.MAX_HUB_TRACKING_X;
  }

  // Periodic & logging loops

  @Override
  public void periodic() {
    visualize();
    Logger.recordOutput("IN NEUTRAL ZONE", inNeutralZone());
    Logger.recordOutput("IN ALLIANCE ZONE", inAllianceZone());
  }

  // Visualize the robot state, log modes, etc
  public void visualize() {
    // Log robot modes
    Logger.recordOutput("RobotState/Modes/Flywheel Mode", flywheelMode);
    Logger.recordOutput("RobotState/Modes/Intake Mode", intakeMode);
    Logger.recordOutput("RobotState/Modes/Turret Mode", turretMode);

    Logger.recordOutput("RobotState/Drivetrain/Robot Pose", robotFieldPose);
    Logger.recordOutput("RobotState/Drivetrain/Robot Velocity", robotVelocity);
    Logger.recordOutput("RobotState/Drivetrain/Robot Acceleration", robotAcceleration);

    Pose3d climberPose =
        new Pose3d(CLIMBER_ATTACH_OFFSET.getTranslation(), CLIMBER_ATTACH_OFFSET.getRotation())
            .transformBy(
                new Transform3d(
                    new Translation3d(0, 0, -this.climberHeight.in(Meters)), new Rotation3d()));

    climberLigament2d.setLength(climberHeight.in(Centimeters) + 103.5);

    // Climber
    Logger.recordOutput("RobotState/Climber/" + key, climberPose);

    // Turret
    Logger.recordOutput(
        "RobotState/Turret/Turret Field Location Actual",
        RobotState.instance()
            .getRobotPose3d()
            .plus(
                ShooterConstants.ROBOT_TO_TURRET_CENTER.plus(
                    new Transform3d(
                        0, 0, 0, new Rotation3d(0, 0, turretRotationAngle.in(Radians))))));

    Logger.recordOutput(
        "RobotState/Turret/Turret Motor Rotations", turretRotationAngle.in(Degrees));
  }

  // Mechanism offsets, TODO can be moved to their respective constants files
  private static final Transform3d CLIMBER_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(2.125), Inches.of(-11.5), Inches.of(3.5)),
          new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(90)));
}
