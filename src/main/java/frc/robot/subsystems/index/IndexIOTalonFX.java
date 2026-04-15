// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.HardwareID;
import frc.robot.util.RealSubsystem;

/** Add your docs here. */
public class IndexIOTalonFX extends RealSubsystem implements IndexIO {

  private final TalonFXS feedMotor;
  private final TalonFX spindexMotor;
  private Timer feedTimer = new Timer();

  private final SlewRateLimiter spindexVoltageLimiter = new SlewRateLimiter(8.0);
  private double targetSpindexVoltage = 0.0;

  public IndexIOTalonFX() {
    feedMotor = new TalonFXS(IndexConstants.FEED_MOTOR_ID, HardwareID.MAIN_CANBUS);
    feedMotor.getConfigurator().apply(IndexConstants.FEED_CONFIG);

    spindexMotor = new TalonFX(IndexConstants.SPINDEX_MOTOR_ID, HardwareID.MAIN_CANBUS);
    spindexMotor.getConfigurator().apply(IndexConstants.SPINDEX_CONFIG);

    feedTimer.start();
  }

  protected void configureHardware() {}

  @Override
  public void updateInputs(IndexInputs inputs) {
    inputs.feedRPS = feedMotor.getVelocity().getValue().in(RotationsPerSecond);
    inputs.feedDrawAmps.mut_replace(feedMotor.getSupplyCurrent().getValue());
    inputs.feedVolts.mut_replace(feedMotor.getMotorVoltage().getValue());
    inputs.spindexDrawAmps.mut_replace(spindexMotor.getSupplyCurrent().getValue());
    inputs.secondsSinceLastFeed = feedTimer.get();

    if (feedMotor.getVelocity().getValue().lt(RotationsPerSecond.of(105))) {
      feedTimer.reset();
    }

    spindexMotor.setControl(
        new VoltageOut(spindexVoltageLimiter.calculate(targetSpindexVoltage)).withEnableFOC(true));
  }

  @Override
  public void setFeedMotorVoltage(Voltage volts) {
    feedMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }

  @Override
  public void setSpindexMotorVoltageOld(Voltage volts) {
    // spindexMotor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }

  @Override
  public void setSpindexMotorTargetVoltage(Voltage volts) {
    targetSpindexVoltage = volts.magnitude();
  }

  @Override
  public void setGains(Slot0Configs gains) {
    feedMotor.getConfigurator().apply(gains);
  }

  @Override
  public void setFeedMotorRequest(ControlRequest request) {
    feedMotor.setControl(request);
  }

  @Override
  public double getSecondsSinceLastFeed() {
    return feedTimer.get();
  }
}
