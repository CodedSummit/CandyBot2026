// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorIntake;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends Command {
  private FloorIntake m_floorIntake;
  
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public IntakeCommand(FloorIntake intakeSS) {
      m_floorIntake = intakeSS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_floorIntake.startIntakeWheels();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_floorIntake.stopIntakeWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}