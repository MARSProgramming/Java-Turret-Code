// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class RobotContainer {

  private CommandXboxController controller;
  private Turret turret;
  
  public RobotContainer() {
    controller = new CommandXboxController(0);
    turret = new Turret();
    configureBindings();
    
    
  }

  private void configureBindings() {
    controller.rightBumper().whileTrue(turret.setFlywheelOutput(() -> controller.getRightTriggerAxis()));
    controller.x().whileTrue(turret.setTurretOutput(-0.3));
    controller.b().whileTrue(turret.setTurretOutput(0.3));
    controller.y().whileTrue(turret.setTurretPosition(0));
    controller.leftBumper().whileTrue(turret.setMagazineOutput(() -> controller.getLeftTriggerAxis()));
    controller.a().whileTrue(turret.setTurretPosition(90));
    controller.povUp().whileTrue(turret.setHoodOutput(0.35));
    controller.povDown().whileTrue(turret.setHoodOutput(-0.35));
    controller.povLeft().whileTrue(turret.setHoodPosition(0));
    controller.povRight().whileTrue(turret.setHoodPosition(5));
    controller.rightTrigger(0.1).whileTrue(turret.testShoot(() -> controller.getLeftX(), () -> controller.getLeftY()));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
