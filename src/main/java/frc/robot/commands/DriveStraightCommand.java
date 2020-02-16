/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.Drive.Robot.Theta;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.JoystickSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveStraightCommand extends PIDCommand {
  private DriveSubsystem driveSubsystem;
  public DriveStraightCommand(DriveSubsystem drive, JoystickSubsystem stick) {
    super(
        // The controller that the command will use
        drive.getRobotThetaPID(),
        // This should return the measurement
        drive::getHeading,
        // This should return the setpoint (can also be a constant)
        0,
        // This uses the output
        output -> {
          drive.driveCartesian(stick.getY(), stick.getX(), output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(Theta.poseTolerance, Theta.velTolerance);

    driveSubsystem = drive;
    addRequirements(drive, stick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.zeroHeading();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
