/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.JoystickSubsystem;

public class DriveCartesianCommand extends CommandBase {
  /**
   * Creates a new DriveCartesianCommand.
   */

  DriveSubsytem driveSubsystem;
  JoystickSubsystem joystickSubsystem;

  public DriveCartesianCommand(DriveSubsytem drive, JoystickSubsystem joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = drive;
    joystickSubsystem = joystick;
    addRequirements(drive, joystick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.driveCartesian(
      -joystickSubsystem.getY(),
      joystickSubsystem.getX(),
      joystickSubsystem.getTwist()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveCartesian(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
