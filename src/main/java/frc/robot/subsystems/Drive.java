// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.DriveSignal;
import frc.robot.Kinematics;
import frc.robot.lib.geometry.Twist2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {

  private CANSparkMax rightMotor1 = new CANSparkMax(1,  MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(2,  MotorType.kBrushless);
  private CANSparkMax rightMotor3 = new CANSparkMax(3,  MotorType.kBrushless);

  private CANSparkMax leftMotor1 = new CANSparkMax(4,  MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(5,  MotorType.kBrushless);
  private CANSparkMax leftMotor3 = new CANSparkMax(6,  MotorType.kBrushless);

  public Drive() {
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);
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
    // DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    // double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    // setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
}

// public synchronized void setOpenLoop(DriveSignal signal) {
//   if (mDriveControlState != DriveControlState.OPEN_LOOP) {
//       setBrakeMode(false);

//       System.out.println("Switching to open loop");
//       System.out.println(signal);
//       mDriveControlState = DriveControlState.OPEN_LOOP;
//       mLeftMaster.configNeutralDeadband(0.04, 0);
//       mRightMaster.configNeutralDeadband(0.04, 0);
//   }
//   mPeriodicIO.left_demand = signal.getLeft();
//   mPeriodicIO.right_demand = signal.getRight();
//   mPeriodicIO.left_feedforward = 0.0;
//   mPeriodicIO.right_feedforward = 0.0;
// }    

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
