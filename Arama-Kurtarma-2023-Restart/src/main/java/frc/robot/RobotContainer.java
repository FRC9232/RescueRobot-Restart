// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmBackGoPosition;
import frc.robot.commands.ArmGoPosition;
import frc.robot.commands.ArmPositionFineTuning;
import frc.robot.commands.AutoClimbGetReady;
import frc.robot.commands.AutonomousClimb;
import frc.robot.commands.DefaultArmDrive;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.SavePIDConfigs;
import frc.robot.commands.deneme;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveTrain m_driveTrain = new DriveTrain();
  public final ArmSubsystem m_Arm = new ArmSubsystem();
  
  public final CommandGenericHID m_driverController = new CommandGenericHID(0);
  public final GenericHID m_driverControllerHID = m_driverController.getHID();

  public final JoystickButton j2_button1 = new JoystickButton(m_driverControllerHID, 1);
  public final JoystickButton j2_button2 = new JoystickButton(m_driverControllerHID, 2);
  public final JoystickButton j2_button3 = new JoystickButton(m_driverControllerHID, 3);
  public final JoystickButton j2_button4 = new JoystickButton(m_driverControllerHID, 4);
  public final JoystickButton j2_button5 = new JoystickButton(m_driverControllerHID, 5);
  public final JoystickButton j2_button6 = new JoystickButton(m_driverControllerHID, 6);
  public final POVButton buttonPOVRight = new POVButton(m_driverControllerHID, 90);
  public final POVButton buttonPOVUp = new POVButton(m_driverControllerHID, 0);
  public final POVButton buttonPOVDown = new POVButton(m_driverControllerHID, 180);
  public final POVButton buttonPOVLeft = new POVButton(m_driverControllerHID, 270);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    m_driveTrain.setDefaultCommand(
      
       new DefaultDrive(
        m_driveTrain,
        () -> -m_driverController.getRawAxis(1),
        () -> -m_driverController.getRawAxis(2)));
  }
/* 
  m_Arm.setDefaultCommand(
    new DefaultArmDrive(
      m_Arm,
      () -> -m_driverController.getRawAxis(0),
      () -> -m_driverController.getRawAxis(5)));
  
  }
  */

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  

        j2_button4.onTrue(new AutoClimbGetReady(m_driveTrain, m_Arm, 0.4, 0.6));


        m_driverController.axisGreaterThan(3, 0.1).whileTrue(new ArmGoPosition(m_Arm,50, false));
        m_driverController.axisGreaterThan(4, 0.1).whileTrue(new ArmGoPosition(m_Arm,-50, false));
        j2_button1.onTrue(new ArmGoPosition(m_Arm,-376, true));
        //j2_button2.onTrue(new ArmGoPosition(m_Arm, -970,true));
        //j2_button3.onTrue(new deneme(m_Arm, -376));
        
        
        buttonPOVRight.onTrue(new SavePIDConfigs(m_Arm));
        buttonPOVUp.onTrue(new ArmPositionFineTuning(m_Arm, 0.02));
        buttonPOVDown.onTrue(new ArmPositionFineTuning(m_Arm, -0.02));
        buttonPOVLeft.onTrue(new AutoClimbGetReady(m_driveTrain, m_Arm, 0.8, 0.6));



        
        j2_button2.whileTrue(new DefaultArmDrive(m_Arm, -1.0, "front"));
        j2_button3.onTrue(new DefaultArmDrive(m_Arm, 0.05, "front"));

        j2_button5.
        onTrue(new DefaultArmDrive(m_Arm, 0.05, "back"));
        j2_button6.onTrue(new DefaultArmDrive(m_Arm, -0.4, "back"));

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    Command a = new AutoClimbGetReady(m_driveTrain, m_Arm, 0.4, 0.6);
    Command b = new AutonomousClimb(m_driveTrain, m_Arm);
    return a.andThen(b);
  }

  public Command startCommand() {

    return new AutoClimbGetReady(m_driveTrain, m_Arm, 0.4, 0.6);

  }
}
