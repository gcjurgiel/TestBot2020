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
import frc.robot.subsystems.JoystickSubsystem;

public class DriveStraightCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private JoystickSubsystem joystickSubsytem;
  private PIDController controller;
  private double setPoint = 0.0;

  public DriveStraightCommand(DriveSubsystem drive, JoystickSubsystem stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = drive;
    joystickSubsytem = stick;
    controller = drive.getRobotThetaPID();
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(Theta.poseTolerance, Theta.velTolerance);

    addRequirements(drive, stick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = driveSubsystem.getHeading();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(driveSubsystem.getHeading(), setPoint);
    driveSubsystem.driveCartesian(-joystickSubsytem.getY(), joystickSubsytem.getX(), output / 180 + joystickSubsytem.getTwist());

    if(Math.abs(joystickSubsytem.getTwist()) > 0) {
      setPoint = driveSubsystem.getHeading();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
