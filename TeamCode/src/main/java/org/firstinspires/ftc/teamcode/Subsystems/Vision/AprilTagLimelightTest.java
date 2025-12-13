package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@TeleOp
public class AprilTagLimelightTest {

    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    private double botXmm;
    private double botYmm;
    private double targetX;
    private double targetY;
    private double botHeadingDeg;

//    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight-camera");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(4);
        //4 is all tags combined into one pipeline.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

//    @Override
    public void start() {
        limelight.start();
    }

//    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();

            botXmm = botPose.getPosition().x;
            botYmm = botPose.getPosition().y;
            botHeadingDeg = botPose.getOrientation().getYaw();

            targetX = llResult.getTx();
            targetY = llResult.getTy();
            distance = getDistanceFromTags(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Y", llResult.getTy());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());
            telemetry.addData("X (mm)", botXmm);
            telemetry.addData("Y (mm)", botYmm);
            telemetry.addData("Heading (deg)", botHeadingDeg);
        }
    }

    public double getDistanceFromTags(double ta) {
        double scale = 17537.71;
        return scale / ta;
    }

    public double getBotXmm() {
        return botXmm;
    }

    public double getBotYmm() {
        return botYmm;
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY()  {return targetY;}

    public double getBotHeadingDeg() {
        return botHeadingDeg;
    }

    public double getDis(){ return distance;}
}