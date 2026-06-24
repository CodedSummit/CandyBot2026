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
  private final SparkFlex shooterMotor = new SparkFlex(Constants.ShooterConstants.kShootMotorCanbusID, MotorType.kBrushless);

  private GenericEntry nt_shooterSpeed;
  private GenericEntry nt_shooterGainRPM;
  private GenericEntry nt_shooterRPM;

  private PIDController shooterPID = new PIDController(0.0001, 0, 0);
  private boolean isControllingForRPM = false;
 
  public ShootSubsystem() {
    this.initialize();
  }
  public void initialize() {
    setupShuffleboard();
  }

  public Command runWheelsWithPID(){
    return this.startEnd(()-> spinUpWheelsWithPID(), ()-> stopShooting());
  }

  public void spinUpWheelsWithPID(){
    isControllingForRPM = true;
  }

  public boolean shooterMotorRunning() {
    return (shooterMotor.get() != 0.0);
  }

  public void stopShooterMotor() {
    isControllingForRPM = false;
    shooterPID.reset();
    shooterMotor.set(0);
  }

  public void startShooting() {
    shooterMotor.set(getShooterSpeed());
  }

  public Command Shoot() {
    return this.startEnd(
        () -> shooterMotor.set(getShooterSpeed()), // this is set to negative because the controller is inverted
        () -> shooterMotor.set(0));
  }

  @Override
  public void periodic() {
      if(isControllingForRPM){
        double current_rpm = shooterMotor.getEncoder().getVelocity();
        double target_rpm = getWheelsRPM();
        double velocityFF = target_rpm *(0.0001754 * getGain() * 12); //12 because volt control

        shooterPID.setSetpoint(getWheelsRPM());
        double conveyorPID = shooterPID.calculate(current_rpm);
        shooterMotor.setVoltage(MathUtil.clamp(velocityFF + conveyorPID, 0, 14));
      }
  }

  public void stopShooting() {
    isControllingForRPM = false;
    shooterPID.reset();
    shooterMotor.set(0);
  }

  public double getGain() {
    return nt_shooterGainRPM.getDouble(Constants.ShooterConstants.kGainRPM);
  }
  public double getShooterSpeed() {
    return nt_shooterSpeed.getDouble(Constants.ShooterConstants.kShootSpeed);
  }
  public double getWheelsRPM() {
    return nt_shooterRPM.getDouble(Constants.ShooterConstants.kShootRPM);
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    nt_shooterSpeed = tab.addPersistent("Shooter Speed", Constants.ShooterConstants.kShootSpeed)
        .withSize(3, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();

    nt_shooterGainRPM = tab.addPersistent("RPM Gain", Constants.ShooterConstants.kGainRPM)
    .getEntry();

    nt_shooterRPM = tab.addPersistent("Shooter RPM", Constants.ShooterConstants.kShootRPM)
    .getEntry();
    
    tab.add("Shooter Wheels PID", shooterPID);
  }
}