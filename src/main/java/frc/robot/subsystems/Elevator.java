// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkRelativeEncoder
import com.revrobotics.RelativeEncoder;

public class Elevator extends SubsystemBase {
  private SparkFlex elevatorMotor1;
  private SparkFlex elevatorMotor2;
  private RelativeEncoder relativeEncoder;
  private SparkClosedLoopController eleavtorController;

  private SparkFlexConfig leaderConfig = new SparkFlexConfig();
  private SparkFlexConfig followerConfig = new SparkFlexConfig();
    
  private double setpointHeight = 0.0;
  private boolean positionControlEnabled = false;

  public Elevator(){
    elevatorMotor1 = new SparkFlex(10, MotorType.kBrushless);
    elevatorMotor2 = new SparkFlex(11, MotorType.kBrushless);
    elevatorController = elevatorMotor1.getClosedLoopController();
    relativeEncoder = elevatorMotor1.getEncoder();

    leaderConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      .voltageCompensation(12.0)
      .inverted(true);

    leaderConfig.encoder
      .positionConversion(2.0 * Math.PI * Units.inchesToMeters(1.75) / 5.0)
      .velocityConversion((2.0 * Math.PI * Units.inchesToMeters(1.75) / 5.0) / 60);

    leaderConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.83).i(0).d(0)
      .feedForward
        .kS(0.584).kG(0.366).kV(2.15).kA(0.05)
      .maxMotion
        .cruiseVelocity(0.75)
        .maxAcceleration(1.0)
        .allowedProfileError(0.01);

    followerConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(40)
      .voltageCompensation(12.0)
      .inverted(true);
      .follow(elevatorMotor1, true)

    elevatorMotor1.configure(
      leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
    );

    elevatorMotor2.configure(
      followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters
    );

  relativeEncoder.setPosition(0.0);
  }

  public double getElevatorHeight(){
    return relativeEncoder.getPosition();
  }

  public double getElevatorHeight(){
    return relativeEncoder.getVelocity();
  }

  public void setDesiredHeight(double heightMeters){
    setpointHeight = MathUtil.clamp(heightMeters, 0, 1.44);
    elevatorController.setSetpoint(
      setpointHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0
    );
  }

  public void setElevatorSpeed(double speed){
    positionControlEnabled = false;
    elevatorMotor1.setElevatorSpeed(MathUtil.clamp(speed, -1, 1));
  }

  public void setElevatorVoltage(double voltage){
    positionControlEnabled = false;
    elevatorMotor1.setElevatorVoltage(MathUtil.clamp(speed, -12, 12));
  }

  public void stop(){
    elevatorMotor1.stopMotor();
  }

  public void zeroEncoder(){
    elevatorMotor1.stop();
    elevatorMotor1.relativeEncoder(0.0);
    setpointHeight = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}