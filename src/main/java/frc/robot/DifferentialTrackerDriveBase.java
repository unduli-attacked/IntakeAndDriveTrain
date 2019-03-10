package frc.robot;

import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;
import com.team254.lib.physics.DifferentialDrive.WheelState;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.TrajectoryTrackerDriveBase.TrajectoryTrackerOutput;

public abstract class DifferentialTrackerDriveBase extends TrajectoryTrackerDriveBase{
  DifferentialDrive differentialDrive;

  @Override
  void setOutput(TrajectoryTrackerOutput output){

  }

  void setOutputFromKinematics(ChassisState chassisVelocity){
    var wheelVelocities = differentialDrive.solveInverseKinematics(chassisVelocity);
    var feedForwardVoltages = differentialDrive.getVoltagesFromkV(wheelVelocities);

    setOutput(wheelVelocities, feedForwardVoltages);
  }

  void setOutputFromDynamics(ChassisState chassisVelocity, ChassisState chassisAcceleration){
    var dynamics = differentialDrive.solveInverseDynamics(chassisVelocity, chassisAcceleration);

    setOutput(dynamics.getWheelVelocity(), dynamics.getVoltage());
  }

  void setOutput(WheelState wheelVelocities, WheelState wheelVoltages){
    leftMotor.setSpeed(speed);
  }

  private double getSpeedFromVelocity(Encoder motorEncoder)
}