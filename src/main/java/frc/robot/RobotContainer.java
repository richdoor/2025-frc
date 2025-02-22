// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.GoToSetpoint;
import frc.robot.subsystems.ClawSubsystem;

import static frc.robot.Constants.ClawConstants.setpoint1;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClawSubsystem m_claw = new ClawSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController mechXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

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
    mechXbox.leftBumper().whileTrue(m_claw.wheelForward());
    mechXbox.rightBumper().whileTrue(m_claw.wheelBackward());

    mechXbox.a().onTrue(new GoToSetpoint(m_claw, ClawConstants.setpoint1));
    mechXbox.x().onTrue(new GoToSetpoint(m_claw, ClawConstants.setpoint2));
    mechXbox.y().onTrue(new GoToSetpoint(m_claw, ClawConstants.setpoint3));
    mechXbox.b().onTrue(new GoToSetpoint(m_claw, ClawConstants.setpoint4));
    mechXbox.rightTrigger().whileTrue(m_claw.printPosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
