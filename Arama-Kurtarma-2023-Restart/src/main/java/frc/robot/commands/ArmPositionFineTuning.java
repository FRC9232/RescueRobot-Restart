// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;


public class ArmPositionFineTuning extends CommandBase {
  private final ArmSubsystem m_armSystem;
  private final double m_speed;
 
  /** Creates a new MainArmGoPosition. */
  public ArmPositionFineTuning(ArmSubsystem armSystem, double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSystem = armSystem;
    m_speed = d;
    addRequirements(m_armSystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   m_armSystem.manualSpeed += m_speed;
   m_armSystem.setPercent("front", m_armSystem.manualSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("manual speed", m_armSystem.manualSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

   // m_armSystem.setPosition(m_armSystem.mainArmEncoder.getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {                                                   

  return true;
}
}
