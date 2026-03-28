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
  private MutAngle targetTurretAngleWithOmegaLookahead = new MutAngle(0.0, 0.0, Degrees);
  private MutAngle targetTurretAngleActual = new MutAngle(0.0, 0.0, Degrees);

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
  private LoggedTunableNumber accelerationConstantTuning =
      new LoggedTunableNumber(
          "ACCELERATION_COMPENSATION_CONSTANT",
          ShooterConstants.ACCELERATION_COMPENSATION_CONSTANT);
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
    checkShotReadiness();
    log();
  }

  private void log() {
    Logger.recordOutput(
        "Shot Calculator/Target Turret Field Position With Omega lookahead",
        RobotState.instance()
            .getRobotPose3d()
            .plus(
                ShooterConstants.ROBOT_TO_TURRET_CENTER.plus(
                    new Transform3d(
                        0,
                        0,
                        0,
                        new Rotation3d(0, 0, targetTurretAngleWithOmegaLookahead.in(Radians))))));

    Logger.recordOutput("Shot Calculator/Target Turret Angle", targetTurretAngleActual.in(Degrees));
    Logger.recordOutput(
        "Shot Calculator/Target Turret Angle With Omega Lookahead",
        targetTurretAngleWithOmegaLookahead.in(Degrees));
    Logger.recordOutput("Shot Calculator/Target to shoot at", poseToAimAt);

    Logger.recordOutput("Shot Calculator/Shot Checks/Flywheel spooled", flywheelReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Linear velocity", translationalReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Angular velocity", angularReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Turret rotation", turretRotationReadyToShoot);
    Logger.recordOutput("Shot Calculator/Shot Checks/Hood angled", hoodReadyToShoot);
  }

  private void calculateTargetParameters() {
    boolean isRed = RobotContainer.isRed();
    setAimingTarget(isRed);

    // Translational velocity of the robot
    ChassisSpeeds robotVelocity = RobotState.instance().getRobotVelocity();

    // The translational acceleration of the robot
    ChassisSpeeds robotAcceleration = RobotState.instance().getRobotAcceleration();

    // Robot pose in 3d space
    Pose3d robotPose3D = RobotState.instance().getRobotPose3d();

    // Robot pose but with omaga lookahead compensation
    Pose3d robotPose3DWithOmegaLookahead =
        robotPose3D.plus(
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

    // Where the turret is on the field, the rotation being the robot's rotation
    // plus the omega lookahead
    Pose2d turretFieldLocationWithOmegaLookahead =
        robotPose3DWithOmegaLookahead.plus(ShooterConstants.ROBOT_TO_TURRET_CENTER).toPose2d();

    // 384 --92

    // Compensate by adding the velocity of the robot to the target position (by
    // some constants)

    ChassisSpeeds compensatedChassisSpeeds = RobotState.instance().getRobotVelocity();
    compensatedChassisSpeeds =
        compensatedChassisSpeeds.plus(
            new ChassisSpeeds(
                robotAcceleration.vxMetersPerSecond * accelerationConstantTuning.getAsDouble(),
                robotAcceleration.vyMetersPerSecond * accelerationConstantTuning.getAsDouble(),
                robotAcceleration.omegaRadiansPerSecond
                    * accelerationConstantTuning.getAsDouble()));

    Pose2d velocityPose =
        new Pose2d(
                -compensatedChassisSpeeds.vxMetersPerSecond
                    * (LoggingConstants.tuningMode
                        ? velocityConstantTuning.getAsDouble()
                        : ShooterConstants.VELOCITY_COMPENSATION_CONSTANT),
                -compensatedChassisSpeeds.vyMetersPerSecond
                    * (LoggingConstants.tuningMode
                        ? velocityConstantTuning.getAsDouble()
                        : ShooterConstants.VELOCITY_COMPENSATION_CONSTANT),
                Rotation2d.kZero)
            .rotateBy(robotPose3DWithOmegaLookahead.getRotation().toRotation2d());

    Pose2d poseToAimAtCompensated =
        poseToAimAt.plus(
            RobotState.instance().getTurretMode() == TurretMode.TRACKING_HUB
                ? new Transform2d(
                    velocityPose.getX(), velocityPose.getY(), velocityPose.getRotation())
                : new Transform2d(
                    -velocityPose.getX(),
                    -velocityPose.getY(),
                    velocityPose.getRotation().times(-1)));

    // Now the location is compensated for robot velocity. Not good enough! Needs
    // angular rotation. The turret has translational velocity independent of the
    // robot's linear velocity, requiring extra compensation
    // factored in as well
    Vector<N3> omegaVector = VecBuilder.fill(0, 0, compensatedChassisSpeeds.omegaRadiansPerSecond);

    // Vector format of this
    Vector<N3> robotToTurretVector =
        VecBuilder.fill(
            ShooterConstants.ROBOT_TO_TURRET_CENTER.getX(),
            ShooterConstants.ROBOT_TO_TURRET_CENTER.getY(),
            ShooterConstants.ROBOT_TO_TURRET_CENTER.getZ());

    // The turret's translational velocity (independent of the robot)
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
    Vector<N2> vTurret =
        VecBuilder.fill(
            turretFieldLocationWithOmegaLookahead.getX(),
            turretFieldLocationWithOmegaLookahead.getY());
    // Vector from blue origin to compensated target
    Vector<N2> vTarget =
        VecBuilder.fill(poseToAimAtCompensated.getX(), poseToAimAtCompensated.getY());
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
            .minus(Radians.of(robotPose3DWithOmegaLookahead.getRotation().getZ()))
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
    // turret will either under or over rotate if the error vector is not parallel
    // to i. But that is fixed now ok?

    targetTurretAngleWithOmegaLookahead.mut_replace(potAngle);

    double turretToCompensatedTargetMagnitude =
        (getDistance(
            robotPose3DWithOmegaLookahead,
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
      } else if (RobotState.instance().getTurretMode() == TurretMode.PASSING) {
        targetHoodExtension.mut_replace(Millimeters.of(50));
        targetFlywheelVelocity.mut_replace(
            getTargetVeloPassing(turretToCompensatedTargetMagnitude));
      } else {
        targetHoodExtension.mut_replace(
            getTargetExtensionLongHoodLow(turretToCompensatedTargetMagnitude));
        targetFlywheelVelocity.mut_replace(
            getTargetVeloLongHoodLow(turretToCompensatedTargetMagnitude));
      }
    }
  }

  private void checkShotReadiness() {
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
                Math.pow(RobotState.instance().getRobotVelocity().vxMetersPerSecond, 2)
                    + Math.pow(RobotState.instance().getRobotVelocity().vyMetersPerSecond, 2)),
            ShooterConstants.TRANSLATIONAL_SPEED_THRESHOLD.in(MetersPerSecond));

    angularReadyToShoot =
        MathUtil.isNear(
            0.0,
            RobotState.instance().getRobotVelocity().omegaRadiansPerSecond,
            ShooterConstants.ANGULAR_SPEED_THRESHOLD.in(RotationsPerSecond));

    turretRotationReadyToShoot =
        MathUtil.isNear(
            0.0,
            RobotState.instance()
                .getTurretRotationAngle()
                .minus(targetTurretAngleWithOmegaLookahead)
                .in(Degrees),
            ShooterConstants.TURRET_ANGLE_ERROR_THRESHOLD.in(Degrees));

    hoodReadyToShoot =
        MathUtil.isNear(
            RobotState.instance().getHoodExtension().in(Millimeters),
            targetHoodExtension.in(Millimeters),
            ShooterConstants.HOOD_EXTENSION_THRESHOLD.in(Millimeters));
  }

  private void setAimingTarget(boolean isRed) {
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
                  : FieldConstants.PASSING_LOWER;
        } else {
          poseToAimAt =
              isRed
                  ? FlippingUtil.flipFieldPose(FieldConstants.PASSING_LOWER)
                  : FieldConstants.PASSING_UPPER;
        }
      default:
        break;
    }
  }

  private AngularVelocity getTargetVeloPassing(double distanceInMeters) {
    double rps = MathUtil.clamp(((5.0 * distanceInMeters) + 15), 15, 50);
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
      return Millimeters.of(hoodDistanceTunable.getAsDouble());
    }

    return targetHoodExtension;
  }

  public Angle getTargetTurretAngleWithout() {
    return targetTurretAngleActual;
  }

  public Angle getTargetTurretAngleWithOmegaLookahead() {
    return targetTurretAngleWithOmegaLookahead;
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
