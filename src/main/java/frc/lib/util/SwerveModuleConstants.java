package frc.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    // public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
    //     this.driveMotorID = driveMotorID;
    //     this.angleMotorID = angleMotorID;
    //     this.cancoderID = canCoderID;
    //     this.angleOffset = angleOffset;
    // }

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset,InvertedValue driveMotorInvert,InvertedValue angleMotorInvert) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.driveMotorInvert=driveMotorInvert;
        this.angleMotorInvert=angleMotorInvert;
    }
}
