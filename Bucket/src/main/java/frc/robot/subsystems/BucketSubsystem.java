package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import javax.lang.model.util.ElementScanner14;

//import com.fasterxml.jackson.databind.cfg.ContextAttributes;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class BucketSubsystem extends SubsystemBase {
    private final SparkMax bucketMotor;

    public BucketSubsystem() {
        bucketMotor = new SparkMax(6, MotorType.kBrushless);
        
    }

    public void setPower(double power) {
        bucketMotor.set(power); //sets the power for the bucket
    }

    public void stop() {
        bucketMotor.set(0); //stops the bucket from moving
    }
}