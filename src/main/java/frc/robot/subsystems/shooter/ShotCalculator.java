package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.managersubsystems.RobotState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends VirtualSubsystem {

  private String key;
  private static ShotCalculator instance;

  private MutDistance targetHoodExtension = Millimeters.of(25.0).mutableCopy();
  private MutAngle targetTurretAngle = new MutAngle(0.0, 0.0, Degrees);

  private LoggedTunableNumber targetVelocityManual =
      new LoggedTunableNumber("FLYWHEEL VELO TUNING", 44);
  private LoggedTunableNumber hoodDistanceTunable = new LoggedTunableNumber("HOOD MM TUNING", 20);
  private MutAngularVelocity targetFlywheelVelocity = RotationsPerSecond.of(44.0).mutableCopy();

  private Pose2d poseToAimAt = new Pose2d();

  private boolean flywheelReadyToShoot = false;
  private boolean translationalReadyToShoot = false;
  private boolean angularReadyToShoot = false;
  private boolean turretRotationReadyToShoot = false;
  private boolean hoodReadyToShoot = false;

  public ShotCalculator(String key) {
    this.key = key;
  }

  public static ShotCalculator instance() {
    if (instance == null) {
      instance = new ShotCalculator("calc");
    }
    return instance;
  }

  @Override
  public void periodic() {
    calculateTargetParameters();
    log();
  }

  private void log() {
    Logger.recordOutput(
        "Shot Calculator/Target Turret Field Position",
        RobotState.instance()
            .getRobotPose3d()
            .plus(
                ShooterConstants.ROBOT_TO_TURRET_CENTER.plus(
                    new Transform3d(
                        0, 0, 0, new Rotation3d(0, 0, targetTurretAngle.in(Radians))))));

    Logger.recordOutput("Shot Calculator/Target Turret Angle", targetTurretAngle.in(Degrees));
    Logger.recordOutput("Shot Calculator/Target to shoot at", poseToAimAt);

    Logger.recordOutput("Shot Calculator/Shot Checks/Flywheel spooled", flywheelReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Linear velocity", translationalReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Angular velocity", angularReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Turret rotation", turretRotationReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Hood angled", hoodReadyToShoot);
  }

  private void calculateTargetParameters() {
    boolean isRed = RobotContainer.isRed();

    // Vel
    ChassisSpeeds robotVelocity = RobotState.instance().getRobotVelocity();
    // Robot pose
    Pose2d robotPose =
        RobotState.instance()
            .getRobotPose()
            .plus(
                new Transform2d(
                    Translation2d.kZero,
                    new Rotation2d(
                        Radians.of(
                            robotVelocity.omegaRadiansPerSecond
                                * ShooterConstants.TURRET_ROTATION_COMPENSATION_CONSTANT))));
    // Acc
    ChassisSpeeds robotAcceleration = RobotState.instance().getRobotAcceleration();
    // Where the turret is relative to blue origin
    Pose2d turretFieldLocation =
        RobotState.instance()
            .getRobotPose3d()
            .plus(ShooterConstants.ROBOT_TO_TURRET_CENTER)
            .toPose2d();

    // 384 --92

    // Where the turret is aiming as pose
    switch (RobotState.instance().getTurretMode()) {
      case TRACKING_HUB:
        poseToAimAt = isRed ? FieldConstants.Red.HUB_CENTER : FieldConstants.Blue.HUB_CENTER;
        break;
      default:
        break;
    }

    // Compensate for velocity
    // Target field pose
    // TODO it is a work in progress
    Pose2d poseToAimAtCompensated =
        poseToAimAt.plus(
            new Transform2d(
                -robotVelocity.vxMetersPerSecond / 1.15,
                -robotVelocity.vyMetersPerSecond / 1.15,
                Rotation2d.kZero));
    Logger.recordOutput(
        "Shot Calculator/VELO COMPENSATED Target to shoot at", poseToAimAtCompensated);

    // Vector from blue origin to robot
    Vector<N2> vTurret = VecBuilder.fill(turretFieldLocation.getX(), turretFieldLocation.getY());
    // Vector from blue origin to compensated target
    Vector<N2> vTarget =
        VecBuilder.fill(poseToAimAtCompensated.getX(), poseToAimAtCompensated.getY());
    // Turret to
    Vector<N2> turretToCompensatedTarget = vTarget.minus(vTurret);
    // Cosine component from dot product
    Angle cosAngle = Radians.of(turretToCompensatedTarget.dot(VecBuilder.fill(1, 0)));
    // Vector for the cross product, represents the error vector from the robot
    Vector<N3> turretToTarget3d =
        VecBuilder.fill(turretToCompensatedTarget.get(0), turretToCompensatedTarget.get(1), 0);
    // Sine component from cross product, i crossed with the error vector from robot
    // to target
    Angle sinAngle =
        Radians.of(getMagnitude3d(Vector.cross(VecBuilder.fill(1, 0, 0), turretToTarget3d)));

    // Final field-relative rotation for the turret to track
    Angle potAngle =
        Radians.of(
                Math.atan2(sinAngle.in(Radians), cosAngle.in(Radians))
                    * (turretToCompensatedTarget.get(1) < 0 ? -1.0 : 1.0))
            .minus(Radians.of(robotPose.getRotation().getRadians()))
            .minus(Degrees.of(180));

    if (turretToCompensatedTarget.get(1) < 0) {
      potAngle = potAngle.plus(Degrees.of(360));
    }

    // Now, compensate for angular velocity by adding a linear constant proportional
    // to rotation speed

    if (potAngle.in(Degrees) < -180) {
      potAngle = potAngle.plus(Degrees.of(360));
    }
    if (potAngle.in(Degrees) > 180) {
      potAngle = potAngle.minus(Degrees.of(360));
    }
    potAngle =
        Degrees.of(
            MathUtil.clamp(
                potAngle.in(Degrees),
                ShooterConstants.TURRET_ROTATION_LIMIT_REVERSE.in(Degrees),
                ShooterConstants.TURRET_ROTATION_LIMIT_FORWARD.in(Degrees)));

    // Now we have a target to rotate to on the field. Yet, there is one issue: the
    // turret will wither under or over rotate if the error vector is not parallel
    // to i.

    // This is prob the problem! compensatedAngle & targetTurretAngle
    Logger.recordOutput("Shot Calculator/compensatedAngle", potAngle.in(Degrees));

    targetTurretAngle.mut_replace(potAngle);
    // targetTurretAngle.mut_replace(Degrees.of(45));

    double turretToCompensatedTargetMagnitude =
        (getDistance(
            RobotState.instance().getRobotPose3d(),
            new Pose3d(poseToAimAtCompensated)
                .plus(
                    new Transform3d(
                        0, 0, FieldConstants.HUB_HEIGHT.in(Meters), new Rotation3d()))));

    // If we are not calibrating, calculate the target spool & hood angle
    if (!LoggingConstants.shooterCalibrationMode) {
      targetHoodExtension.mut_replace(
          getTargetExtensionLongHoodLow(turretToCompensatedTargetMagnitude));
      targetFlywheelVelocity.mut_replace(
          getTargetVeloLongHoodLow(turretToCompensatedTargetMagnitude));
    }

    // ----------------------------------------------------------
    // CHECK IF WE ARE READY TO SHOOT
    // ----------------------------------------------------------
    // Velocity and shot readiness
    flywheelReadyToShoot =
        MathUtil.isNear(
            ShotCalculator.instance().getTargetFlywheelVelocity().in(RotationsPerSecond),
            RobotState.instance().getFlywheelVelocity().in(RotationsPerSecond),
            frc.robot.managersubsystems.RobotState.instance().getFlywheelMode()
                    == FlywheelMode.FRENZY
                ? ShooterConstants.FRENZY_FEED_THRESHOLD.in(RotationsPerSecond)
                : ShooterConstants.NORMAL_FEED_THRESHOLD.in(RotationsPerSecond));

    translationalReadyToShoot =
        MathUtil.isNear(
            0.0,
            Math.sqrt(
                Math.pow(robotVelocity.vxMetersPerSecond, 2)
                    + Math.pow(robotVelocity.vyMetersPerSecond, 2)),
            ShooterConstants.TRANSLATIONAL_SPEED_THRESHOLD.in(MetersPerSecond));

    angularReadyToShoot =
        MathUtil.isNear(
            0.0,
            robotVelocity.omegaRadiansPerSecond,
            ShooterConstants.ANGULAR_SPEED_THRESHOLD.in(RotationsPerSecond));

    turretRotationReadyToShoot =
        MathUtil.isNear(
            0.0,
            RobotState.instance().getTurretRotationAngle().minus(targetTurretAngle).in(Degrees),
            ShooterConstants.TURRET_ANGLE_ERROR_THRESHOLD.in(Degrees));

    hoodReadyToShoot =
        MathUtil.isNear(
            RobotState.instance().getHoodExtension().in(Millimeters),
            targetHoodExtension.in(Millimeters),
            ShooterConstants.HOOD_EXTENSION_THRESHOLD.in(Millimeters));
  }

  private AngularVelocity getTargetVelo(double distanceInMeters) {
    // double rps = MathUtil.clamp((3.76 * distanceInMeters) + 30.7, 5, 60);
    double rps = MathUtil.clamp((4.76 * distanceInMeters) + 30.7, 5, 60);
    return RotationsPerSecond.of(rps);
  }

  private Distance getTargetExtension(double distanceInMeters) {
    // double mm = MathUtil.clamp(-31.7 + (48.3 * Math.log(distanceInMeters)), 0,
    // 50.0);
    double mm = (1.0 / (-0.15 * (distanceInMeters - 0.184))) + 40;
    if (distanceInMeters <= 2) {
      mm = 0;
    }
    // double mm = MathUtil.clamp(-21 + (10 * (distanceInMeters)), 0, 50.0);
    return Millimeters.of(MathUtil.clamp(mm, 0, 50));
  }

  private AngularVelocity getTargetVeloLongHoodLow(double distanceInMeters) {
    double rps = MathUtil.clamp(((4.12 * distanceInMeters) + 26), 5, 60);
    return RotationsPerSecond.of(rps);
  }

  private Distance getTargetExtensionLongHoodLow(double distanceInMeters) {
    double mm = (1.9 * distanceInMeters) + 33.9;
    return Millimeters.of(MathUtil.clamp(mm, 0, 50));
  }

  // ------------------------------------------------------------------------------
  // Getters
  // ------------------------------------------------------------------------------
  public AngularVelocity getTargetFlywheelVelocity() {

    if (LoggingConstants.shooterCalibrationMode && targetVelocityManual.hasChanged(hashCode())) {
      this.targetFlywheelVelocity.mut_replace(
          RotationsPerSecond.of(targetVelocityManual.getAsDouble()));
    }

    return targetFlywheelVelocity;
  }

  public Distance getTargetHoodExtension() {

    if (LoggingConstants.shooterCalibrationMode && hoodDistanceTunable.hasChanged(hashCode())) {
      this.targetHoodExtension.mut_replace(Millimeters.of(hoodDistanceTunable.getAsDouble()));
    }

    return targetHoodExtension;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
  }

  // TODO not all values are used here for sake of testing
  public boolean getReadyToFeed() {
    return flywheelReadyToShoot
        && translationalReadyToShoot
        && angularReadyToShoot
        && turretRotationReadyToShoot;
  }

  // ------------------------------------------------------------------------------
  // Helper functions
  // ------------------------------------------------------------------------------
  private double getMagnitude2d(Vector<N2> vector) {
    return Math.sqrt(Math.pow(vector.get(0), 2.0) + Math.pow(vector.get(1), 2.0));
  }

  private double getMagnitude3d(Vector<N3> vector) {
    return Math.sqrt(
        Math.pow(vector.get(0), 2.0) + Math.pow(vector.get(1), 2.0) + Math.pow(vector.get(2), 2.0));
  }

  private double getDistance(Pose3d p1, Pose3d p2) {
    return Math.sqrt(
        Math.pow(p1.getX() - p2.getX(), 2)
            + Math.pow(p1.getY() - p2.getY(), 2)
            + Math.pow(p1.getZ() - p2.getZ(), 2));
  }
}
