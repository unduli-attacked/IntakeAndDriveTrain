package frc.robot;

import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;
import com.team254.lib.physics.DifferentialDrive.WheelState;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.TrajectoryTrackerDriveBase.TrajectoryTrackerOutput;

public interface DifferentialTrackerDriveBase extends TrajectoryTrackerDriveBase{
  DifferentialDrive getDifferentialDrive();

  // @Override
  default void setOutput(org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput output){
    setOutputFromDynamics(output.getDifferentialDriveVelocity(), output.getDifferentialDriveAcceleration());
  }

  default void setOutputFromKinematics(ChassisState chassisVelocity){
    var wheelVelocities = getDifferentialDrive().solveInverseKinematics(chassisVelocity);
    var feedForwardVoltages = getDifferentialDrive().getVoltagesFromkV(wheelVelocities);

    setOutput(wheelVelocities, feedForwardVoltages);
  }

  default void setOutputFromDynamics(ChassisState chassisVelocity, ChassisState chassisAcceleration){
    var dynamics = getDifferentialDrive().solveInverseDynamics(chassisVelocity, chassisAcceleration);

    setOutput(dynamics.getWheelVelocity(), dynamics.getVoltage());
  }

  default void setOutput(WheelState wheelVelocities, WheelState wheelVoltages){
    // DifferentialDrive.WheelState wheelVelocities;
    // wheelVoltages: DifferentialDrive.WheelState
// ) {

    var leftSpeed = VelocityKt.getVelocity(LengthKt.getMeter((wheelVelocities.getLeft() * getDifferentialDrive().getWheelRadius())));

    var rightSpeed = VelocityKt.getVelocity(LengthKt.getMeter((wheelVelocities.getRight() * getDifferentialDrive().getWheelRadius())));


    getLeftMotor().setVelocityAndArbitraryFeedForward(
        leftSpeed,
        wheelVoltages.getLeft() / 12.0
    );
    getRightMotor().setVelocityAndArbitraryFeedForward(
        rightSpeed,
        wheelVoltages.getRight() / 12.0
    );

  }

}