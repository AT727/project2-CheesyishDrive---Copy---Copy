package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.util.Util;
import frc.robot.lib.util.DriveSignal;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.lib.drivers.LazySparkMax;
import frc.robot.lib.drivers.SparkMaxFactory;
import frc.robot.lib.geometry.Twist2d;
import com.revrobotics.CANSparkMax.IdleMode;

public class DriveCommand extends CommandBase {
  private final Drive mdrive;

 
  public DriveCommand(Drive drive) {
    mdrive = drive;

    addRequirements(drive);
  }

  public DriveCommand getCheesyDrive() {
    double throttle = -deadband(cheesyController.getY(Hand.kLeft), 0.1);
    double wheel = -deadband(cheesyController.getX(Hand.kRight), 0.1);
    boolean quickTurn = cheesyController.getBumper(Hand.kRight);

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveVelocity signal = KinematicsUtils.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.leftVelocity), Math.abs(signal.rightVelocity)));
    return new DriveCommand(signal.leftVelocity / scaling_factor, signal.rightVelocity / scaling_factor);
  }

  @Override
  public void initialize() {}

 
  @Override
  public void execute() {}


  @Override
  public void end(boolean interrupted) {}

 
  @Override
  public boolean isFinished() {
    return false;
  }
}
