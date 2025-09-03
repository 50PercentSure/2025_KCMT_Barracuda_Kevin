package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, SparkLowLevel.MotorType.kBrushless);

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
            ShooterConstants.shooterMotorFFKs,
            ShooterConstants.shooterMotorFFKv,
            ShooterConstants.shooterMotorFFKa
    );

    private final PIDController shooterPID = new PIDController(
            ShooterConstants.ShooterMotorP,
            ShooterConstants.ShooterMotorI,
            ShooterConstants.ShooterMotorD
    );

    public ShooterSubsystem() {

    }

    public void setShooterMotorRPM(double rpm) {
        shooterMotor.setVoltage(
                shooterPID.calculate(MathUtils.RPMtoRadians(shooterMotor.getEncoder().getVelocity()), MathUtils.RPMtoRadians(rpm)) +
                        shooterFF.calculate(MathUtils.RPMtoRadians(rpm))
        );
    }

    public void runVolts(double volts) {
        shooterMotor.setVoltage(volts);
    }

    public void resetPID() {
        shooterPID.reset();
    }
}
