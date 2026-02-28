package frc.robot.managersubsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.enums.Modes.ClimbCameraMode;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.IntakeDeployMode;
import frc.robot.enums.Modes.IntakeSpinMode;
import frc.robot.enums.Modes.TurretMode;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.VirtualSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class RobotState extends VirtualSubsystem {

  // Robot Modes
  private IntakeDeployMode intakeDeployMode = IntakeDeployMode.IDLE_RETRACTED;
  private IntakeSpinMode intakeSpinMode = IntakeSpinMode.IDLE;
  private FlywheelMode flywheelMode = FlywheelMode.MANUAL;
  private TurretMode turretMode = TurretMode.IDLE;
  private ClimbCameraMode climbCameraMode = ClimbCameraMode.CLIMB_RED;

  private static RobotState instance;

  private final String key;

  // Intake
  private boolean intakeFullyRetracted = false;

  // Shooter
  private MutAngle turretRotationAngle = Degrees.of(0.0).mutableCopy();
  private MutAngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0).mutableCopy();
  private MutDistance hoodExtension = Millimeters.of(0.0).mutableCopy();

  // Driving data
  private Pose2d robotFieldPose = Pose2d.kZero;
  private ChassisSpeeds robotVelocity = new ChassisSpeeds(0, 0, 0);
  private ChassisSpeeds robotAcceleration = new ChassisSpeeds(0, 0, 0);

  private Pose3d turretFieldPosition = new Pose3d();

  private double turretToHubMagnitude = 2.54;

  private RobotState(String key) {
    this.key = key;
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

  public double getRobotSpeedMPS() {
    return Math.sqrt(
        Math.pow(robotVelocity.vxMetersPerSecond, 2)
            + Math.pow(robotVelocity.vyMetersPerSecond, 2));
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
  // If robot is test mode, guard against automatic controls

  public IntakeDeployMode getIntakeDeployMode() {
    return intakeDeployMode;
  }

  public void setIntakeDeployMode(IntakeDeployMode intakeMode) {
    if (edu.wpi.first.wpilibj.RobotState.isTest()) this.intakeDeployMode = IntakeDeployMode.MANUAL;
    else this.intakeDeployMode = intakeMode;
  }

  public IntakeSpinMode getIntakeSpinMode() {
    return intakeSpinMode;
  }

  public void setIntakeSpinMode(IntakeSpinMode intakeMode) {
    if (edu.wpi.first.wpilibj.RobotState.isTest()) this.intakeSpinMode = IntakeSpinMode.MANUAL;
    else this.intakeSpinMode = intakeMode;
  }

  public FlywheelMode getFlywheelMode() {
    return flywheelMode;
  }

  public void setFlywheelMode(FlywheelMode flywheelMode) {
    if (edu.wpi.first.wpilibj.RobotState.isTest()) this.flywheelMode = FlywheelMode.MANUAL;
    else this.flywheelMode = flywheelMode;
  }

  public TurretMode getTurretMode() {
    return turretMode;
  }

  public void setTurretMode(TurretMode turretMode) {
    if (edu.wpi.first.wpilibj.RobotState.isTest()) this.turretMode = TurretMode.MANUAL;
    else this.turretMode = turretMode;
  }

  public ClimbCameraMode getClimbCameraMode() {
    return climbCameraMode;
  }

  public void setClimbCameraMode(ClimbCameraMode climbCameraMode) {
    this.climbCameraMode = climbCameraMode;
  }

  // Subsystems are below
  // Climber getters & setters ------------------------------------------------

  // Turret getters & setters --------------------------------------------------
  public Angle getTurretRotationAngle() {
    return turretRotationAngle;
  }

  public void setFlywheelVelocity(AngularVelocity angle) {
    flywheelVelocity.mut_replace(angle);
  }

  public AngularVelocity getFlywheelVelocity() {
    return flywheelVelocity;
  }

  public void setTurretRotationAngle(Angle angle) {
    turretRotationAngle.mut_replace(angle);
  }

  public Distance getHoodExtension() {
    return hoodExtension;
  }

  public void setHoodExtension(Distance d) {
    hoodExtension.mut_replace(d);
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
    Logger.recordOutput("RobotState/Modes/Intake Deploy Mode", intakeDeployMode);
    Logger.recordOutput("RobotState/Modes/Intake Spin Mode", intakeSpinMode);
    Logger.recordOutput("RobotState/Modes/Turret Mode", turretMode);

    Logger.recordOutput("RobotState/Drivetrain/Robot Pose", robotFieldPose);
    Logger.recordOutput("RobotState/Drivetrain/Robot Velocity", robotVelocity);
    Logger.recordOutput("RobotState/Drivetrain/Robot Acceleration", robotAcceleration);

    Logger.recordOutput("RobotState/Intake fully retracted", intakeFullyRetracted);

    turretFieldPosition =
        RobotState.instance()
            .getRobotPose3d()
            .plus(
                ShooterConstants.ROBOT_TO_TURRET_CENTER.plus(
                    new Transform3d(
                        0, 0, 0, new Rotation3d(0, 0, turretRotationAngle.in(Radians)))));

    // Turret
    Logger.recordOutput("RobotState/Turret/Turret Field Location Actual", turretFieldPosition);

    turretToHubMagnitude =
        (getDistance(
            turretFieldPosition,
            new Pose3d(
                    RobotContainer.isRed()
                        ? FieldConstants.Red.HUB_CENTER
                        : FieldConstants.Blue.HUB_CENTER)
                .plus(
                    new Transform3d(
                        0, 0, FieldConstants.HUB_HEIGHT.in(Meters), new Rotation3d()))));

    Logger.recordOutput("RobotState/Turret/Turret to top hub", turretToHubMagnitude);

    Logger.recordOutput(
        "RobotState/Turret/Turret Motor Rotations", turretRotationAngle.in(Degrees));
  }

  public double getTurretToHubMagnitude() {
    return turretToHubMagnitude;
  }

  public Pose3d getTurretFieldPosition() {
    return turretFieldPosition;
  }

  public void setIntakeFullyRetracted(boolean val) {
    intakeFullyRetracted = val;
  }

  public BooleanSupplier getIntakeFullyRetractedSupplier() {
    return () -> intakeFullyRetracted;
  }

  // Mechanism offsets, TODO can be moved to their respective constants files
  private static final Transform3d CLIMBER_ATTACH_OFFSET =
      new Transform3d(
          new Translation3d(Inches.of(2.125), Inches.of(-11.5), Inches.of(3.5)),
          new Rotation3d(Degrees.of(180), Degrees.of(0), Degrees.of(90)));

  private double getDistance(Pose3d p1, Pose3d p2) {
    return Math.sqrt(
        Math.pow(p1.getX() - p2.getX(), 2)
            + Math.pow(p1.getY() - p2.getY(), 2)
            + Math.pow(p1.getZ() - p2.getZ(), 2));
  }
}
