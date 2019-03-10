package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

public class Constants {

    public static final int kLeftMotor = 0;
    public static final int kRightMotor = 1;

    public static final int kLeftEncoderA = 0;
    public static final int kLeftEncoderB = 1;
    public static final int kRightEncoderA = 2;
    public static final int kRightEncoderB = 3;

    public static final boolean kLeftEncoderInvert = false;
    public static final boolean kRightEncoderInvert = false;

    public static final Length kWheelDiameter = LengthKt.getInch(6);

}