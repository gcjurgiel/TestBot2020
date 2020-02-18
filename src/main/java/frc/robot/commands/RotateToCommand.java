/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drive.Robot.Theta;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private PIDController controller;
  private double setPoint = 0.0;
  private double targetAngle;

  public RotateToCommand(double target, DriveSubsystem drive) {
    driveSubsystem = drive;
    controller = drive.getRobotThetaPID();
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(Theta.poseTolerance, Theta.velTolerance);
    targetAngle = target;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = (((targetAngle + driveSubsystem.getHeading() + 180) % 360) - 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(driveSubsystem.getHeading(), setPoint);
    driveSubsystem.driveCartesian(0.0, 0.0, output / 180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveCartesian(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
