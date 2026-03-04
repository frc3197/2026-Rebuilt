package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.util.FlippingUtil;
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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LoggingConstants;
import frc.robot.enums.Modes.FlywheelMode;
import frc.robot.enums.Modes.TurretMode;
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

  private LoggedTunableNumber omegaConstantTuning =
      new LoggedTunableNumber(
          "TURRET_OMEGA_COMPENSATION_CONSTANT",
          ShooterConstants.TURRET_OMEGA_COMPENSATION_CONSTANT);
  private LoggedTunableNumber velocityConstantTuning =
      new LoggedTunableNumber(
          "VELOCITY_COMPENSATION_CONSTANT", ShooterConstants.VELOCITY_COMPENSATION_CONSTANT);
  private LoggedTunableNumber rotationLookaheadConstantTuning =
      new LoggedTunableNumber(
          "TURRET_ROTATION_LOOKAHEAD_CONSTANT",
          ShooterConstants.TURRET_ROTATION_LOOKAHEAD_CONSTANT);

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
                                * (LoggingConstants.tuningMode
                                    ? rotationLookaheadConstantTuning.getAsDouble()
                                    : ShooterConstants.TURRET_ROTATION_LOOKAHEAD_CONSTANT)))));
    Pose3d robotPose3D =
        RobotState.instance()
            .getRobotPose3d()
            .plus(
                new Transform3d(
                    Translation3d.kZero,
                    new Rotation3d(
                        0,
                        0,
                        Radians.of(
                                robotVelocity.omegaRadiansPerSecond
                                    * (LoggingConstants.tuningMode
                                        ? rotationLookaheadConstantTuning.getAsDouble()
                                        : ShooterConstants.TURRET_ROTATION_LOOKAHEAD_CONSTANT))
                            .in(Radians))));
    // Acc
    ChassisSpeeds robotAcceleration = RobotState.instance().getRobotAcceleration();
    // Where the turret is relative to blue origin
    Pose2d turretFieldLocation =
        robotPose3D
            .plus(ShooterConstants.ROBOT_TO_TURRET_CENTER)
            .toPose2d()
            .plus(
                new Transform2d(
                    Translation2d.kZero,
                    new Rotation2d(
                        Radians.of(
                            robotVelocity.omegaRadiansPerSecond
                                * (LoggingConstants.tuningMode
                                    ? rotationLookaheadConstantTuning.getAsDouble()
                                    : ShooterConstants.TURRET_ROTATION_LOOKAHEAD_CONSTANT)))));

    // 384 --92

    // Where the turret is aiming as pose
    switch (RobotState.instance().getTurretMode()) {
      case TRACKING_HUB:
        poseToAimAt = isRed ? FieldConstants.Red.HUB_CENTER : FieldConstants.Blue.HUB_CENTER;
        break;
      case PASSING:
        if (RobotState.instance().getRobotPose().getY() < FieldConstants.Blue.HUB_CENTER.getY()) {
          poseToAimAt =
              isRed
                  ? FlippingUtil.flipFieldPose(FieldConstants.PASSING_UPPER)
                  : FieldConstants.PASSING_UPPER;
        } else {
          poseToAimAt =
              isRed
                  ? FlippingUtil.flipFieldPose(FieldConstants.PASSING_LOWER)
                  : FieldConstants.PASSING_UPPER;
        }
      default:
        break;
    }

    // Compensate for velocity
    // Target field pose
    // TODO it is a work in progress
    Pose2d velocityPose =
        new Pose2d(
                -robotVelocity.vxMetersPerSecond
                    * (LoggingConstants.tuningMode
                        ? velocityConstantTuning.getAsDouble()
                        : ShooterConstants.VELOCITY_COMPENSATION_CONSTANT),
                -robotVelocity.vyMetersPerSecond
                    * (LoggingConstants.tuningMode
                        ? velocityConstantTuning.getAsDouble()
                        : ShooterConstants.VELOCITY_COMPENSATION_CONSTANT),
                Rotation2d.kZero)
            .rotateBy(RobotState.instance().getRobotPose().getRotation());

    Pose2d poseToAimAtCompensated =
        poseToAimAt.plus(
            new Transform2d(velocityPose.getX(), velocityPose.getY(), velocityPose.getRotation()));

    // Now the location is compensated for robot velocity. Not good enough! Needs
    // angular rotation
    // factored in as well
    Vector<N3> omegaVector = VecBuilder.fill(0, 0, robotVelocity.omegaRadiansPerSecond);
    Pose3d robotToTurretWithAngle =
        new Pose3d(
            ShooterConstants.ROBOT_TO_TURRET_CENTER.getTranslation(),
            ShooterConstants.ROBOT_TO_TURRET_CENTER.getRotation());
    robotToTurretWithAngle = robotToTurretWithAngle.rotateBy(robotPose3D.getRotation());

    Vector<N3> robotToTurretVector =
        VecBuilder.fill(
            robotToTurretWithAngle.getX(),
            robotToTurretWithAngle.getY(),
            robotToTurretWithAngle.getZ());
    SmartDashboard.putString(
        "ROBOT TO TURRET WITH ROT COMP",
        "" + robotToTurretVector.get(0) + ", " + robotToTurretVector.get(1));
    Vector<N3> turretVelocityVector = Vector.cross(omegaVector, robotToTurretVector);
    turretVelocityVector =
        turretVelocityVector.times(
            LoggingConstants.tuningMode
                ? omegaConstantTuning.getAsDouble()
                : ShooterConstants.TURRET_OMEGA_COMPENSATION_CONSTANT);
    poseToAimAtCompensated =
        poseToAimAtCompensated.plus(
            new Transform2d(
                -turretVelocityVector.get(0), -turretVelocityVector.get(1), new Rotation2d()));

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
            robotPose3D,
            new Pose3d(poseToAimAtCompensated)
                .plus(
                    new Transform3d(
                        0, 0, FieldConstants.HUB_HEIGHT.in(Meters), new Rotation3d()))));

    // If we are not calibrating, calculate the target spool & hood angle
    if (!LoggingConstants.shooterCalibrationMode) {
      if (RobotState.instance().getTurretMode() == TurretMode.TRACKING_HUB) {
        targetHoodExtension.mut_replace(
            getTargetExtensionLongHoodLow(turretToCompensatedTargetMagnitude));
        targetFlywheelVelocity.mut_replace(
            getTargetVeloLongHoodLow(turretToCompensatedTargetMagnitude));
      } else {
        targetHoodExtension.mut_replace(Millimeters.of(55));
        targetFlywheelVelocity.mut_replace(
            getTargetVeloPassing(turretToCompensatedTargetMagnitude));
      }
    }

    // ----------------------------------------------------------
    // CHECK IF WE ARE READY TO SHOOT
    // ----------------------------------------------------------
    // Velocity and shot readiness
    flywheelReadyToShoot =
        MathUtil.isNear(
            ShotCalculator.instance().getTargetFlywheelVelocity().in(RotationsPerSecond),
            RobotState.instance().getFlywheelVelocity().in(RotationsPerSecond),
            RobotState.instance().getFlywheelMode() == FlywheelMode.FRENZY
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

  private AngularVelocity getTargetVeloPassing(double distanceInMeters) {
    double rps = MathUtil.clamp(((3.05 * distanceInMeters) + 25), 15, 50);
    return RotationsPerSecond.of(rps);
  }

  private AngularVelocity getTargetVeloLongHoodLow(double distanceInMeters) {
    double rps = MathUtil.clamp(((4.12 * distanceInMeters) + 26), 5, 80);
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
        && turretRotationReadyToShoot
        && (RobotState.instance().getFlywheelMode() == FlywheelMode.FRENZY
            || (translationalReadyToShoot && angularReadyToShoot));
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
