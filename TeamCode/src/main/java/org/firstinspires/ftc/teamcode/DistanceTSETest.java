package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "DistanceTest", group = "A")
public class DistanceTSETest extends LinearOpMode {
    protected DistanceSensor left_dist;
    protected DistanceSensor front_dist;
    protected DistanceSensor back_dist;



    @Override
    public void runOpMode() {

        left_dist = hardwareMap.get(DistanceSensor.class,"left_dist");
        front_dist = hardwareMap.get(DistanceSensor.class,"front_dist");
        back_dist = hardwareMap.get(DistanceSensor.class,"back_dist");



        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Dist value (MM) ",left_dist.getDistance(DistanceUnit.MM));
            telemetry.addData("Right Dist value (MM)", front_dist.getDistance(DistanceUnit.MM));
            telemetry.addData("Back Dist value (MM)", back_dist.getDistance(DistanceUnit.MM));
            telemetry.addLine("400 MS refresh rate per read");
            telemetry.update();


        }
    }
}









