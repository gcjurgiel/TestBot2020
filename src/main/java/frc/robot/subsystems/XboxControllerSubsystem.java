/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Controller;;

public class XboxControllerSubsystem extends SubsystemBase {
  private static final XboxController controller = new XboxController(Controller.XboxController.port);

  public XboxControllerSubsystem() {
  }

  public double getRightX() {
    return (Math.abs(controller.getX(Hand.kRight)) <= 0.2) ? 0.0 : controller.getX(Hand.kRight);
  }

  public double getRawRightX() {
    return controller.getX(Hand.kRight);
  }

  public double getRightY() {
    return (Math.abs(controller.getY(Hand.kRight)) <= 0.2) ? 0.0 : controller.getY(Hand.kRight);    
  }

  public double getRawRightY() {
    return controller.getY(Hand.kRight);    
  }

  public double getLeftX() {
    return (Math.abs(controller.getX(Hand.kLeft)) <= 0.2) ? 0.0 : controller.getX(Hand.kLeft);  
  }

  public double getRawLeftX() {
    return controller.getX(Hand.kLeft);  
  }

  public double getLeftY() {
    return (Math.abs(controller.getY(Hand.kLeft)) <= 0.2) ? 0.0 : controller.getY(Hand.kLeft);
  }

  public double getRawLeftY() {
    return controller.getY(Hand.kLeft);
  }

  public XboxController getController() {
    return controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
