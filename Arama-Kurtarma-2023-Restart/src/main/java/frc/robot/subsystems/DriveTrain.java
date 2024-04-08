// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  public WPI_VictorSPX right = new WPI_VictorSPX(Constants.Drive.RIGHT);
	public WPI_VictorSPX left = new WPI_VictorSPX(Constants.Drive.LEFT);
  public int driveMode;
  public int stepCount = 0;
  public AHRS gyro = new AHRS(SPI.Port.kMXP);

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    right.setInverted(false);
    left.setInverted(true);
    gyro.calibrate();
    gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Yaw", gyro.getYaw());
    SmartDashboard.putNumber("Pitch", gyro.getPitch());
  }

  public void driveIt(){
    left.set(ControlMode.PercentOutput, 0.6);
    right.set(ControlMode.PercentOutput, 0.6);

  }

  public void teleopArcadeDrivePercent(double power, double turn){

    left.set(ControlMode.PercentOutput, power + turn);
    right.set(ControlMode.PercentOutput, power-turn);
  }
  
  
}
