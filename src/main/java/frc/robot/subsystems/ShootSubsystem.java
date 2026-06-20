// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

@Logged
public class ShootSubsystem extends SubsystemBase {
  private GenericEntry nt_intakeRPM;
  private final SparkFlex intakeWheels = new SparkFlex(canbus id);

  private final DigitalInput floorInput = new DigitalInput(0);
  private final DutyCycleEncoder intakeArmPosition = new DutyCycleEncoder(diginput, 360, offset);

  private PIDController intakeWheelsPID = new PIDController(0.0001, 0, 0);
  private boolean isControllingForRPM = false;
 
  public ShootSubsystem() {
    this.initialize();
  }

  public void initialize() {}

  public Command runWheelsWithPID(){
    return this.startEnd(()-> spinUpWheelsWithPID(), ()-> stopIntake());
  }

  public void spinUpWheelsWithPID(){
    isControllingForRPM = true;
  }

  public Command ManualRunIntake() {
    return runOnce(() -> intakeWheels.set(getIntakeSpeed()));
  }

  public Command ManualStopIntake() {
    return runOnce(() -> intakeWheels.set(0));
  }

  public boolean intakeWheelsRunning() {
    return (intakeWheels.get() != 0.0);
  }

  public void stopIntakeWheels() {
    isControllingForRPM = false;
    intakeWheelsPID.reset();
    intakeWheels.set(0);
  }

  public void startIntakeWheels() {
    intakeWheels.set(getIntakeSpeed());
  }

  public Command Intake() {
    return this.startEnd(
        () -> intakeWheels.set(getIntakeSpeed()), // this is set to negative because the controller is inverted
        () -> intakeWheels.set(0));
  }

  @Logged
  public Command Outtake() {
    return this.startEnd(
        () -> intakeWheels.set(-getIntakeSpeed()),
        () -> intakeWheels.set(0));
  }

  @Override
  public void periodic() {
      if(isControllingForRPM){
        double current_rpm = intakeWheels.getEncoder().getVelocity();
        double target_rpm = getWheelsRPM();
        double velocityFF = target_rpm *(0.0001754 * getGain() * 12); //12 because volt control

        intakeWheelsPID.setSetpoint(getWheelsRPM());
        double wheelsPID = intakeWheelsPID.calculate(current_rpm);
        intakeWheels.setVoltage(MathUtil.clamp(velocityFF + wheelsPID, 0, 14));
      }
  }

  public void stopIntake() {
    isControllingForRPM = false;
    intakeWheelsPID.reset();
    intakeWheels.set(0);
  }

  public double getWheelsRPM() {
    return nt_intakeRPM.getDouble(intake rpm);
  }
}