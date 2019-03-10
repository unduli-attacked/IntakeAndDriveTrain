package frc.robot;

import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;

import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSpeedController;

public abstract class TrajectoryTrackerDriveBase {
  HalfBakedEncodedPWMMotorController leftMotor, rightMotor;

  Pose2d robotPosition;

  TrajectoryTracker tracker;

  abstract void setOutput(TrajectoryTrackerOutput output);

  void zeroOutputs(){
    leftMotor.setRaw(0);
    rightMotor.setRaw(0);
  }


  class TrajectoryTrackerOutput{
    Velocity<Length> linearV;
    Acceleration<Length> linearA;
    Velocity<Rotation2d> angularV;
    Acceleration<Rotation2d> angularA;
    DifferentialDrive.ChassisState differentialDriveVelocity, differentialDriveAcceleration;
    public TrajectoryTrackerOutput(double _linearV, double _linearA, double _angularV, double _angularA){
      this.linearV = VelocityKt.getVelocity(LengthKt.getMeter(_linearV));
      this.linearA = AccelerationKt.getAcceleration(LengthKt.getMeter(_linearA));

      this.angularV = VelocityKt.getVelocity(Rotation2dKt.getRadian(_angularV));
      this.angularA = AccelerationKt.getAcceleration(Rotation2dKt.getRadian(_angularA));

      this.differentialDriveAcceleration = new DifferentialDrive.ChassisState(_linearA, _angularA);
      this.differentialDriveVelocity = new DifferentialDrive.ChassisState(_linearV, _angularV);
    }

    public TrajectoryTrackerOutput(Velocity<Length> lV, Acceleration<Length> lA, Velocity<Rotation2d> aV, Acceleration<Rotation2d> aA){
      this(lV.getValue(), lA.getValue(), aV.getValue(), aA.getValue());
    }


    public Velocity<Length> getLinearVelocity(){
      return this.linearV;
    }

    public Acceleration<Length> getLinearAcceleration(){
      return this.linearA;
    }

    public Velocity<Rotation2d> getAngularVelocity(){
      return this.angularV;
    }

    public Acceleration<Rotation2d> getAngularAcceleration(){
      return this.angularA;
    }

    public ChassisState getDifferentialDriveAcceleration(){
      return this.differentialDriveAcceleration;
    }

    public ChassisState getDifferentialDriveVelocity(){
      return this.differentialDriveVelocity;
    }
  }

}