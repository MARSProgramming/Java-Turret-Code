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
    controller.rightTrigger(0.1).whileTrue(turret.shootball(0.7, 0.4));
    controller.leftTrigger(0.1).whileTrue(turret.setMagazineOutput(0.3));
    controller.povUp().whileTrue(turret.setHoodOutput(0.2));
    controller.povDown().whileTrue(turret.setHoodOutput(-0.2));
    controller.povLeft().whileTrue(turret.setHoodPosition(4000));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

}
