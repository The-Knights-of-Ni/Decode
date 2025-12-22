package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

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
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public AprilTagLimelightTest(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }


//    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "limelight-camera");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(4);
        //4 is all tags combined into one pipeline.
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        //Something here in init is throwing an error that's making it not work with teleop
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
            List<LLResultTypes.FiducialResult> res = llResult.getFiducialResults();
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
            telemetry.addData("Results Size", res.size());
            for(int i = 0; i<res.size(); i++){
                telemetry.addData("Result[i] fudicial id = ", res.get(i).getFiducialId());
                telemetry.addData("Result[i] family = ", res.get(i).getFamily());
            }
        }
        else{
            telemetry.addLine("No results :(");
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