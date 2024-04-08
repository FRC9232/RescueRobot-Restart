// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class AutonomousClimbArm extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_DriveTrain;
  private final ArmSubsystem m_Arm;
  private double pitch;
  private double climbPositionForFront = -1800;
  private double climbPositionForBack = -2100;

  public AutonomousClimbArm(DriveTrain driveTrain, ArmSubsystem armSubsystem) {
    m_DriveTrain = driveTrain;
    m_Arm = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
    pitch = m_DriveTrain.gyro.getPitch();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  public void updateGyro(){
    pitch = m_DriveTrain.gyro.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_DriveTrain.driveMode == 1){
      if(pitch >= 7){
        m_DriveTrain.driveMode = 2;
      }
    }
    if(m_DriveTrain.driveMode == 2){

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
