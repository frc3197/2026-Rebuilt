// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.enums.MatchPeriod;
import java.util.Optional;
import java.util.function.BooleanSupplier;

public class MatchTimeUtil extends VirtualSubsystem {

  private static MatchTimeUtil instance;
  private final String key;
  private MatchPeriod matchPeriod = MatchPeriod.DISABLED;

  private Optional<Boolean> wonAuto = Optional.empty();
  private boolean isAuto = false;

  private double shiftSwitchThreshold = 5.25;

  private MatchTimeUtil(String key) {
    this.key = key;
  }

  public static MatchTimeUtil instance() {
    if (instance == null) {
      instance = new MatchTimeUtil("measured");
    }
    return instance;
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Current Match Period", matchPeriod.toString());

    double time = DriverStation.getMatchTime();
    isAuto = DriverStation.isAutonomous();

    if (time < 0) {
      matchPeriod = MatchPeriod.DISABLED;
      return;
    }

    String gameData;
    gameData = DriverStation.getGameSpecificMessage();
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B':
          wonAuto = Optional.of(!RobotContainer.isRed());
          break;
        case 'R':
          wonAuto = Optional.of(RobotContainer.isRed());
          break;
        default:
          wonAuto = Optional.empty();
          DriverStation.reportError("INVALID AUTO WINNER", false);
          break;
      }
    } else {
      wonAuto = Optional.empty();
    }

    if (isAuto && time >= 0.0) {
      matchPeriod = MatchPeriod.AUTO;
      return;
    }

    if (time >= 130 + shiftSwitchThreshold) {
      matchPeriod = MatchPeriod.TRANSITION;
      return;
    }

    if (time >= 130) {
      if (wonAuto.isPresent() && wonAuto.get()) {
        matchPeriod = MatchPeriod.ABOUT_TO_BE_INACTIVE;
      } else {
        matchPeriod = MatchPeriod.ABOUT_TO_BE_ACTIVE;
      }
      return;
    }

    if (time <= 30) {
      matchPeriod = MatchPeriod.ENDGAME;
      return;
    }

    double minusEndgame = time - 30.0;
    int period = (int) Math.floor(minusEndgame / 25.0);
    boolean isActiveCurrently;
    if ((wonAuto.isPresent() && wonAuto.get())) {
      isActiveCurrently = (period == 0 || period == 2);
    } else {
      isActiveCurrently = (period == 3 || period == 1);
    }

    if (!isActiveCurrently && minusEndgame <= shiftSwitchThreshold) {
      matchPeriod = MatchPeriod.ABOUT_TO_BE_ACTIVE;
      return;
    }

    if ((minusEndgame % 25.0) < shiftSwitchThreshold) {
      matchPeriod =
          isActiveCurrently ? MatchPeriod.ABOUT_TO_BE_INACTIVE : MatchPeriod.ABOUT_TO_BE_ACTIVE;
      return;
    }

    matchPeriod = isActiveCurrently ? MatchPeriod.ACTIVE : MatchPeriod.INACTIVE;
  }

  public Optional<Boolean> wonAuto() {
    return wonAuto;
  }

  public MatchPeriod getMatchPeriod() {
    return matchPeriod;
  }

  public BooleanSupplier aboutToBecomeActiveSupplier() {
    return () -> matchPeriod == MatchPeriod.ABOUT_TO_BE_ACTIVE && !DriverStation.isTest();
  }
}
