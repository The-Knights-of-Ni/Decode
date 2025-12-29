package org.firstinspires.ftc.teamcode.Subsystems.Control;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.FeedForward;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PID;
import org.firstinspires.ftc.teamcode.Subsystems.Subsystem;
//import org.firstinspires.ftc.teamcode.Util.ServoEx;


/**
 * Control subsystem for controlling arms and claws
 */
public class Control extends Subsystem {
//    public final Servo shootFlap;
    public final DcMotorEx shootMotor;
    public final DcMotorEx intakeMotor;
    public final DcMotorEx turretMotor;
    public final Servo lift;

    private final FeedForward feedFowardVelocity = new FeedForward(10, 10);
    private final PID PIDVelocity = new PID(0.002, 0, 0.0002);

    public Control(Telemetry telemetry, DcMotorEx intakeMotor, DcMotorEx shootMotor, DcMotorEx  turretMotor, Servo lift) {
        super(telemetry, "control");
        this.shootMotor = shootMotor;
        this.intakeMotor = intakeMotor;
        this.turretMotor = turretMotor;
        this.lift = lift;
    }

    /**
     * Gets all defaults, directions,etc. ready for the autonomous period
     */
    public void initDevicesAuto() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setDirection(DcMotor.Direction.FORWARD);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /**
     * Gets all defaults, directions,etc. ready for the teleop period
     */
    public void initDevicesTeleop() {
        shootMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startIntake() {
//        intakeMotor.setMotorEnable();
        intakeMotor.setPower(-1);
    }

    public void stopIntake() {
        intakeMotor.setMotorDisable();
        intakeMotor.setPower(0);

    }

    public void startShoot(double power) {
        shootMotor.setMotorEnable();
        shootMotor.setPower(-power);
    }


    public void stopShoot() {
        shootMotor.setMotorDisable();
        shootMotor.setPower(0);
    }

    public void holdShootVelocity(double targetVelocity) {
        double currentVelocity = shootMotor.getVelocity();
        double feedForward = feedFowardVelocity.calculate(targetVelocity, targetVelocity-currentVelocity+0.1);
        double PIDCorrect = PIDVelocity.calculate(targetVelocity, currentVelocity);

        double variable = feedForward + PIDCorrect;

        shootMotor.setVelocity(variable);
    }

}