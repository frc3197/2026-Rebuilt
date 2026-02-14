package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import frc.robot.RobotContainer;
import frc.robot.managersubsystems.RobotState;
import frc.robot.util.FieldConstants;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class ShotCalculator extends VirtualSubsystem {

  private String key;
  public static ShotCalculator instance;

  private MutAngle targetHoodAngle = new MutAngle(45.0, 45.0, Degrees);
  private MutAngle targetTurretAngle = new MutAngle(0.0, 0.0, Degrees);
  private MutAngularVelocity targetFlywheelVelocity =
      new MutAngularVelocity(0.0, 0.0, RotationsPerSecond);

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

    Logger.recordOutput("Shot Calculator/Robot to hub angle", targetTurretAngle);
  }

  private void calculateTargetParameters() {
    boolean isRed = RobotContainer.isRed();

    Pose2d robotPose = RobotState.instance().getRobotPose();
    ChassisSpeeds robotVelocity = RobotState.instance().getRobotVelocity();
    ChassisSpeeds robotAcceleration = RobotState.instance().getRobotAcceleration();

    // Where the turret is aiming
    Pose2d turretTargetPose =
        isRed ? FieldConstants.Red.HUB_CENTER : FieldConstants.Blue.HUB_CENTER;

    Logger.recordOutput("Shot Calculator/Target to shoot at", turretTargetPose);

    // Where the turret is relative to blue origin
    Pose2d turretFieldLocation =
        RobotState.instance()
            .getRobotPose3d()
            .plus(ShooterConstants.ROBOT_TO_TURRET_CENTER)
            .toPose2d();

    // Vector from blue origin to robot
    Vector<N2> vRobot = VecBuilder.fill(turretFieldLocation.getX(), turretFieldLocation.getY());

    // Vector from blue origin to target
    Vector<N2> vTarget = VecBuilder.fill(turretTargetPose.getX(), turretTargetPose.getY());
    Vector<N2> robotToHub = vTarget.minus(vRobot);

    // Log the error vector
    Logger.recordOutput(
        "Shot Calculator/Robot to Hub", "<" + robotToHub.get(0) + ", " + robotToHub.get(1) + ">");

    // Cosine component from dot product
    Angle cosAngle = Radians.of(robotToHub.dot(VecBuilder.fill(1, 0)));

    // Sine component from cross product
    Vector<N3> robotToHub3d = VecBuilder.fill(robotToHub.get(0), robotToHub.get(1), 0);
    Angle sinAngle =
        Radians.of(getMagnitude3d(Vector.cross(VecBuilder.fill(1, 0, 0), robotToHub3d)));

    // Final target rotation for the turret to track
    Angle potAngle =
        Radians.of(
                Math.atan2(sinAngle.in(Radians), cosAngle.in(Radians))
                    * (robotToHub.get(1) < 0 ? -1.0 : 1.0))
            .minus(Radians.of(robotPose.getRotation().getRadians()));

    if (robotToHub.get(1) < 0) {
      potAngle = potAngle.plus(Degrees.of(360));
    }

    targetTurretAngle.mut_replace(potAngle);
    // targetTurretAngle.mut_replace(Degrees.of(45));
  }

  // Getters ---------------------------------------------------------------
  public Angle getTargetHoodAngle() {
    return targetHoodAngle;
  }

  public Angle getTargetTurretAngle() {
    return targetTurretAngle;
  }

  public AngularVelocity getTargetFlywheelVelocity() {
    return targetFlywheelVelocity;
  }

  // Helper functions ------------------------------------------------------
  private double getMagnitude2d(Vector<N2> vector) {
    return Math.sqrt(Math.pow(vector.get(0), 2.0) + Math.pow(vector.get(1), 2.0));
  }

  private double getMagnitude3d(Vector<N3> vector) {
    return Math.sqrt(
        Math.pow(vector.get(0), 2.0) + Math.pow(vector.get(1), 2.0) + Math.pow(vector.get(2), 2.0));
  }
}
