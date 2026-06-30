// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final SlewRateLimiter throttleLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

  private final ShootSubsystem shootSubsystem = new ShootSubsystem();
  private final HopperSubsystem hopperSubsystem = new HopperSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    m_robotDrive.setDefaultCommand(
      new RunCommand(
        () -> m_robotDrive.curvatureDrive(
          rotationLimiter.calculate(m_robotDrive.squareInput(m_driverController.getRightX())),
          throttleLimiter.calculate(m_robotDrive.squareInput(m_driverController.getLeftY())),
          true
        ), m_robotDrive
      )
    );
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
    m_driverController.a().onTrue(new ParallelCommandGroup(
      shootSubsystem.toggleShoot(),
      hopperSubsystem.toggleSpin()
    ));
    
    m_driverController.rightTrigger()
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(1.25)))
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
    m_driverController.leftTrigger()
      .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(.5)))
      .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;//Autos.exampleAuto(m_exampleSubsystem);
  }
}
