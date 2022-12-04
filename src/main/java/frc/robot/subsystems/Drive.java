// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.lib.drivers.LazySparkMax;
import frc.robot.lib.drivers.SparkMaxFactory;
import frc.robot.lib.geometry.Twist2d;
import com.revrobotics.CANSparkMax.IdleMode;

public class Drive extends SubsystemBase {
  
  private final LazySparkMax mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
  private boolean mIsBrakeMode;
  private DriveControlState mDriveControlState;
  private PeriodicIO mPeriodicIO;

  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
  }

  public Drive() {
    mLeftMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kLeftDriveMasterId);
    configureSpark(mLeftMaster, true, true);

    mLeftSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.kLeftDriveSlaveId, mLeftMaster);
    configureSpark(mLeftSlave, true, false);

    mRightMaster = SparkMaxFactory.createDefaultSparkMax(Constants.kRightDriveMasterId);
    configureSpark(mRightMaster, false, true);

    mRightSlave = SparkMaxFactory.createPermanentSlaveSparkMax(Constants.kRightDriveSlaveId, mRightMaster);
    configureSpark(mRightSlave, false, false);
  }

  private void configureSpark(LazySparkMax sparkMax, boolean left, boolean master) {
    sparkMax.setInverted(!left);
    sparkMax.enableVoltageCompensation(12.0);
    sparkMax.setClosedLoopRampRate(Constants.kDriveVoltageRampRate);
}

  public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
    if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
        throttle = 0;
    }

    if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
        wheel = 0.0;
    }

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.2;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
}

public synchronized void setOpenLoop(DriveSignal signal) {
  if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      setBrakeMode(true);
      System.out.println("switching to open loop");
      System.out.println(signal);
      mDriveControlState = DriveControlState.OPEN_LOOP;
  }

  mPeriodicIO.left_demand = signal.getLeft();
  mPeriodicIO.right_demand = signal.getRight();
  mPeriodicIO.left_feedforward = 0.0;
  mPeriodicIO.right_feedforward = 0.0;
}


public synchronized void setBrakeMode(boolean shouldEnable) {
  if (mIsBrakeMode != shouldEnable) {
      mIsBrakeMode = shouldEnable;
      IdleMode mode = shouldEnable ? IdleMode.kBrake : IdleMode.kCoast;
      mRightMaster.setIdleMode(mode);
      mRightSlave.setIdleMode(mode);

      mLeftMaster.setIdleMode(mode);
      mLeftSlave.setIdleMode(mode);

  }
}

public static class PeriodicIO {
  // INPUTS
  public double timestamp;
  public double left_voltage;
  public double right_voltage;
  public int left_position_ticks;
  public int right_position_ticks;
  public double left_distance;
  public double right_distance;
  public int left_velocity_ticks_per_100ms;
  public int right_velocity_ticks_per_100ms;

  // OUTPUTS
  public double left_demand;
  public double right_demand;
  public double left_accel;
  public double right_accel;
  public double left_feedforward;
  public double right_feedforward;
}

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
