package frc.robot;

import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;
import com.team254.lib.physics.DifferentialDrive.WheelState;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.TrajectoryTrackerDriveBase.TrajectoryTrackerOutput;

public abstract class DifferentialTrackerDriveBase extends TrajectoryTrackerDriveBase{
  DifferentialDrive differentialDrive;

  @Override
  void setOutput(TrajectoryTrackerOutput output){
    setOutputFromDynamics(output.differentialDriveVelocity, output.differentialDriveAcceleration);
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
    // DifferentialDrive.WheelState wheelVelocities;
    // wheelVoltages: DifferentialDrive.WheelState
// ) {

    var leftSpeed = VelocityKt.getVelocity(LengthKt.getMeter((wheelVelocities.getLeft() * differentialDrive.getWheelRadius())));

    var rightSpeed = VelocityKt.getVelocity(LengthKt.getMeter((wheelVelocities.getRight() * differentialDrive.getWheelRadius())));


    leftMotor.setVelocityAndArbitraryFeedForward(
        leftSpeed,
        wheelVoltages.getLeft() / 12.0
    );
    rightMotor.setVelocityAndArbitraryFeedForward(
        rightSpeed,
        wheelVoltages.getRight() / 12.0
    );

  }

}