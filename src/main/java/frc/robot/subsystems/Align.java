// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Align extends SubsystemBase {

  // Controllers for each axis
  private PIDController dController = new PIDController(3.0, 0, 0);
  private PIDController lController = new PIDController(3.0, 0, 0);
  private PIDController thetaController = new PIDController(0.045, 0, 0);

  // Min & max speeds
  private final double maxDepthSpeed = 2.5;
  private final double maxLateralSpeed = 2.5;
  private final double maxRotSpeed = 1.5;

  // Tunable proportional gains
  private LoggedTunableNumber dKp =
      new LoggedTunableNumber("Align/Depth controller proportional", dController.getP());
  private LoggedTunableNumber lKp =
      new LoggedTunableNumber("Align/Lateral controller proportional", lController.getP());

  public Align() {
    dController.setP(dKp.getAsDouble());
    lController.setP(lKp.getAsDouble());
  }

  @Override
  public void periodic() {

    // Check if gains have changed, update appropriate controller
    if (dKp.hasChanged(hashCode())) dController.setP(dKp.getAsDouble());
    if (lKp.hasChanged(hashCode())) lController.setP(lKp.getAsDouble());
  }

  // Get a transformation matrix about the origin of a certain angle
  private Matrix<N2, N2> getRotationTransformationMatrix(Angle angle) {
    return MatBuilder.fill(
        Nat.N2(),
        Nat.N2(),
        Math.cos(angle.in(Radians)),
        -Math.sin(angle.in(Radians)),
        Math.sin(angle.in(Radians)),
        Math.cos(angle.in(Radians)));
  }

  // Public facing align function
  public ChassisSpeeds alignWithTarget(Pose2d target, Pose2d current) {

    // Log the taget align pose while aligning
    Logger.recordOutput("Align/Target Pose", target);

    // Offset pose between the target and current
    Pose2d offsetPose =
        new Pose2d(
            current.getX() - target.getX(),
            current.getY() - target.getY(),
            new Rotation2d(current.getRotation().getRadians() - target.getRotation().getRadians()));

    // Angle of target position
    Angle theta = Degrees.of(target.getRotation().getDegrees());

    // Transformation matrix, converts x/y system to D/L system
    Matrix<N2, N2> transformationMatrix = getRotationTransformationMatrix(theta);

    // Inverse of the transformation matrix, converts D/L system to x/y system
    Matrix<N2, N2> inverseTransformationMatrix = transformationMatrix.inv();

    // Offset between the desired & current poses, in standard x/y coord system
    Vector<N2> errorVector = VecBuilder.fill(offsetPose.getX(), offsetPose.getY());

    // Matrix that yields vector of d & l, think of it as values of (x, y) but in
    // the D/L system
    Matrix<N2, N1> matrixDL = inverseTransformationMatrix.times(errorVector);

    // Robot relative vector for ideal path to target, assuming target rotation
    // Depth in x direction, lateral in y direction
    Vector<N2> targetAlignedOffsets =
        VecBuilder.fill(
            MathUtil.clamp(
                dController.calculate(matrixDL.get(0, 0)), -maxDepthSpeed, maxDepthSpeed),
            MathUtil.clamp(
                lController.calculate(matrixDL.get(1, 0)), -maxLateralSpeed, maxLateralSpeed));

    // Rotation value
    AngularVelocity rotSpeed =
        DegreesPerSecond.of(
            -MathUtil.clamp(
                thetaController.calculate(
                    target.getRotation().minus(current.getRotation()).getDegrees()),
                -maxRotSpeed,
                maxRotSpeed));

    // Log rotation speed while aligning
    Logger.recordOutput("Align/Align rotation speed", rotSpeed.in(DegreesPerSecond));

    // Return field centric chassis speeds from d & l vectors, angle of target, and
    // rotation speed
    return getFieldCentricSpeeds(
        VecBuilder.fill(targetAlignedOffsets.get(0, 0), 0),
        VecBuilder.fill(0, targetAlignedOffsets.get(1, 0)),
        theta,
        rotSpeed);
  }

  // Get field centric speeds from d & l vectors
  private ChassisSpeeds getFieldCentricSpeeds(
      Vector<N2> d, Vector<N2> l, Angle theta, AngularVelocity rotSpeed) {
    Matrix<N2, N2> transformationMatrix = getRotationTransformationMatrix(theta);

    // Convert d & l to cartesian system
    Matrix<N2, N1> dVector = transformationMatrix.times(d);
    Matrix<N2, N1> lVector = transformationMatrix.times(l);

    // Add the vectors for the final cartesian vector
    Matrix<N2, N1> combinedVector = dVector.plus(lVector);

    return new ChassisSpeeds(
        MathUtil.applyDeadband(combinedVector.get(0, 0), 0.05),
        MathUtil.applyDeadband(combinedVector.get(1, 0), 0.05),
        MathUtil.applyDeadband(rotSpeed.in(DegreesPerSecond), 0.05));
  }
}
