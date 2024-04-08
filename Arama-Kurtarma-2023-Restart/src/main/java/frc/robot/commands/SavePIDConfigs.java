// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SavePIDConfigs extends CommandBase {
  private final ArmSubsystem m_armSystem;
  /** Creates a new SavePIDConfigs. */
  public SavePIDConfigs(ArmSubsystem armSystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSystem = armSystem;
    addRequirements(m_armSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSystem.frontArm.config_kF(0, m_armSystem.kF);
    m_armSystem.frontArm.config_kP(0, m_armSystem.kP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
