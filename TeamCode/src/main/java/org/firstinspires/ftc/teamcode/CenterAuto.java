package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "AutoMeet2", group = "A")


public class CenterAuto extends LinearOpMode {

   // private ElapsedTime runtime = new ElapsedTime();

    boolean setup_ready=false;
   // boolean debuging_mode=false;// Need turn off for game
    long delay=0;
   Pose2d pp;

    Baseauto rbga;

    Action tr1;
    Action tr2;
    Action tr3;

    Action tb1;
    Action tb2;
    Action tb3;

    Action pre2;
    Action pre1;
    Action pre3;
    Action bpre1;
    Action bpre2;
    Action bpre3;
    Action bturn1back;
    Action turn1back;
    int path=3;





    Pose2d start=new Pose2d(0,0,Math.toRadians(0));
    Pose2d r1= new Pose2d(2.5,0,Math.toRadians(11));
    Pose2d r2= new Pose2d(4.5,0,Math.toRadians(0));
    Pose2d r3= new Pose2d(2.5,0,Math.toRadians(-9)); // -10.5

    Pose2d b1= new Pose2d(2.5,0,Math.toRadians(11));
    Pose2d b2= new Pose2d(4.5,0,Math.toRadians(0));
    Pose2d b3= new Pose2d(2.5,0,Math.toRadians(-10.5));
   Pose2d r22=new Pose2d(4.5,-5,Math.toRadians(-80));

   Pose2d bpreout2 = new Pose2d(30,50,Math.toRadians(-83));  //66
   Pose2d bpreout1 = new Pose2d(49,50,Math.toRadians(-84));
   Pose2d bpreout3 = new Pose2d(60, -50, Math.toRadians(-82)); //80
   Pose2d rpreout2 = new Pose2d(68, -50, Math.toRadians(82));
   Pose2d rpreout3 = new Pose2d(53, -50,Math.toRadians(82));

   Pose2d t1back = new Pose2d(2.5,0,Math.toRadians(0));
   Pose2d bt1back = new Pose2d(2.5,0,Math.toRadians(0));

   Pose2d rpreout1 = new Pose2d(89, -50, Math.toRadians(84)); // 91

//    Pose2d x = new Pose2d(4.5,0,Math.toRadians(0));





    @Override
    public void runOpMode() {
        rbga = new Baseauto(this, start);

        rbga.baseblue = false;
//        rbga.baseright = false;


      // MecanumDrive drive = new MecanumDrive(super.hardwareMap, start);


//        rbga.front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rbga.front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rbga.rear_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rbga.rear_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        rbga.front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rbga.front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rbga.rear_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rbga.rear_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        rbga.front_right.setDirection(DcMotorSimple.Direction.FORWARD);
//        rbga.rear_right.setDirection(DcMotorSimple.Direction.FORWARD);



        tr1 = rbga.actionBuilder(start)
                .lineToX(2.5)
                .turn(Math.toRadians(11))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
                .build();

        tb1 = rbga.actionBuilder(start)
                .lineToX(2.5)
                .turn(Math.toRadians(11))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
                .build();




        turn1back = rbga.actionBuilder(r1)
                .turn(Math.toRadians(-11))
                .build();

        bturn1back = rbga.actionBuilder(b3)
                .turn(Math.toRadians(25))// 10.5 // 10.5
                .build();
        tr2 = rbga.actionBuilder(start)
//                .splineToLinearHeading(r2, 0)
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
                .lineToX(4.5)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
                .build();

        tb2 = rbga.actionBuilder(start)
                .lineToX(4.5)
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
                .build();

        tr3 = rbga.actionBuilder(start)
                .lineToX(2.5)
                .turn(Math.toRadians(-9))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
                .build();

        tb3 = rbga.actionBuilder(start)
                .lineToX(2.5)
                .turn(Math.toRadians(-10.5))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .setTangent(Math.toRadians(0))
//                .lineToX(32)
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3)
                .build();


       pre2 = rbga.actionBuilder(r2)
               .splineToLinearHeading(rpreout2,0)
               .build();
       bpre2 = rbga.actionBuilder(b2)
                .splineToSplineHeading(bpreout2,0)
                .build();

       pre1 = rbga.actionBuilder(r1)
                       .splineToLinearHeading(rpreout1,0)
                        .build();

       

        bpre1 = rbga.actionBuilder(b1)
                .splineToLinearHeading(bpreout1,0)
                .build();

       pre3 = rbga.actionBuilder(r3)
                       .splineToLinearHeading(rpreout3,0)
                               .build();
        bpre3 = rbga.actionBuilder(b3)
                .splineToLinearHeading(bpreout3,0)
                .build();








        while (!isStopRequested() && !setup_ready){
            if (gamepad1.right_bumper){
                rbga.bottom_claw.setPosition(rbga.bottom_claw_hold);
                sleep(500);
                rbga.top_claw.setPosition(rbga.top_claw_hold);
                sleep(3000);
                rbga.slide(rbga.arm_slide_collapse);
                rbga.arm_handle.setPosition(rbga.arm_handle_idle);
                sleep(1500);
                setup_ready = true;


            }

            if (gamepad1.cross){
                rbga.baseblue = true;
                telemetry.addLine("Chosen Blue");
            }

            if (gamepad1. circle){
                rbga.baseblue = false;
                telemetry.addLine("Chosen Red");
            }
            telemetry.update();


        }

        waitForStart();
      //  rbga.imu.resetYaw();
      //  sleep(300);


        while (opModeIsActive()) {
            telemetry.addData("IMU", rbga.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
            telemetry.update();

            rbga.slide(rbga.arm_slide_extend);

            sleep(1000);
            rbga.arm_handle.setPosition(rbga.handle_dist_det);
            sleep(2500);

            TSE_front_det();

            telemetry.update();
            sleep(500);
            rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
            sleep(1000);
            TSE_left_det();
            sleep(500);

            rbga.slide(rbga.arm_slide_collapse);
            rbga.arm_handle.setPosition(rbga.arm_handle_idle);
            sleep(2500);



            if (path == 1){
                if (rbga.baseblue){
                    bautoroute1();
                }
                else{
                    autoroute1();
                }

            }
            else {
                if (path == 2) {
                    telemetry.addLine("Path driving 2");
                    telemetry.update();
                    sleep(500);
                    if (rbga.baseblue){
                        bautoroute2();
                    }
                    else{
                        autoroute2();
                    }
                }
                else{
                    if (rbga.baseblue){
                        bautoroute3();
                    }
                    else{
                        autoroute3();
                    }
                }
            }



//            autoroute1();
//            autoroute2();
            rbga.stop_drive();
            sleep(300);

            rbga.bottom_claw.setPosition(rbga.bottom_claw_idle);
            sleep(500);

//            if (rbga.baseblue){
//                rbga.stop_drive();
//                return;
//
//            }
            rbga.slide(rbga.arm_slide_collapse);
            rbga.arm_handle.setPosition(rbga.arm_handle_idle);
            sleep(1500);
            rbga.rotate(rbga.arm_rotate_op1-600);
            sleep(2000);



            telemetry.addData("IMU", rbga.imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES)));
            telemetry.update();
            if (path == 1){
                if (rbga.baseblue == true){
                    bautoroutepre1();
                }
                else{
                    turn1back();
                    autoroutepre1();

                }

            }
            else if (path == 2){
                if (rbga.baseblue == true){
                    bautoroutepre2();
                }
                else{
                    autoroutepre2();

                }
            }
            else{
                if (rbga.baseblue == true){
                    bturn1back();
                    bautoroutepre3();
                }
                else{
                    autoroutepre3();

                }
            }
            sleep(300);

            rbga.stop_drive();

            sleep(500);
            rbga.rotate(rbga.arm_rotate_op1-200);
            sleep(250);

            rbga.slide(rbga.arm_slide_collapse+100);

            rbga.arm_handle.setPosition(rbga.arm_handle_op1);
            sleep(250);

            rbga.outtakeauto();
//            rbga.autoback();



//            autoroute3();














            // Step through the list of detected tags and look for a matching tag
//            List<AprilTagDetection> currentDetections = rbga.aprilTag.getDetections();
//            for (AprilTagDetection detection : currentDetections) {
//                // Look to see if we have size info on this tag.
//                if (detection.metadata != null) {
//                    //  Check to see if we want to track towards this tag.
//                    if ((rbga.DESIRED_TAG_ID < 0) || (detection.id == rbga.DESIRED_TAG_ID)) {
//                        // Yes, we want to use this tag.
//                        rbga.targetFound = true;
//                        rbga.desiredTag = detection;
//                        break;  // don't look any further.
//                    } else {
//                        // This tag is in the library, but we do not want to track it right now.
//                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
//                    }
//                } else {
//                    // This tag is NOT in the library, so we don't have enough information to track to it.
//                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
//                }
//            }
//
//            // Tell the driver what we see, and what to do.
//            if (rbga.targetFound) {
//                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
//                telemetry.addData("Found", "ID %d (%s)", rbga.desiredTag.id, rbga.desiredTag.metadata.name);
//                telemetry.addData("Range",  "%5.1f inches", rbga.desiredTag.ftcPose.range);
//                telemetry.addData("Bearing","%3.0f degrees", rbga.desiredTag.ftcPose.bearing);
//                telemetry.addData("Yaw","%3.0f degrees", rbga.desiredTag.ftcPose.yaw);
//            } else {
//                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
//            }
//
//            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
//            if ( targetFound) {
//
//                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
//                double  rangeError      = (rbga.desiredTag.ftcPose.range - rbga.DESIRED_DISTANCE);
//                double  headingError    = rbga.desiredTag.ftcPose.bearing;
//                double  yawError        = desiredTag.ftcPose.yaw;
//
//                // Use the speed and turn "gains" to calculate how we want the robot to move.
//                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
//                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
//                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
//
//                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            } else {
//
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            }
//            telemetry.update();
//
//            // Apply desired axes motions to the drivetrain.
//            moveRobot(drive, strafe, turn);
//            sleep(10);



            sleep(30000);







            break;
        }

    }


    public void  TSE_left_det(){
        sleep(300);

        double left_d = rbga.left_dist.getDistance(DistanceUnit.MM);

        if (left_d > rbga.left_dist_low && left_d < rbga.left_dist_high){
            path = 1;
        }


    }

    public void TSE_front_det(){
        sleep(500);

        if (rbga.front_dist.getDistance(DistanceUnit.MM) < rbga.front_dist_high){
            path = 2;

        }


    }

    void autoroutepre3(){
        Actions.runBlocking(
                new SequentialAction(
                        pre3
                )
        );

        rbga.stop_drive();

    }
    void bautoroutepre3(){
        Actions.runBlocking(
                new SequentialAction(
                        bpre3
                )
        );

        rbga.stop_drive();

    }

    void autoroutepre2(){
        Actions.runBlocking(
                new SequentialAction(
                        pre2
                )
        );

        rbga.stop_drive();

    }

    void bautoroutepre2(){
        Actions.runBlocking(
                new SequentialAction(
                        bpre2
                )
        );

        rbga.stop_drive();

    }

    void autoroutepre1(){
        Actions.runBlocking(
                new SequentialAction(
                        pre1
                )
        );

        rbga.stop_drive();

    }
    void bautoroutepre1(){
        Actions.runBlocking(
                new SequentialAction(
                        bpre1
                )
        );

        rbga.stop_drive();

    }

    void autoroute1() {
        Actions.runBlocking(
                new SequentialAction(
                        tr1

                )
        );

        rbga.stop_drive();


        sleep(500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
        rbga.slide(rbga.arm_slide_extend);
        sleep(1500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
        rbga.slide(rbga.arm_slide_extend-200);
        sleep(500);



    }

    void bautoroute1() {
        Actions.runBlocking(
                new SequentialAction(
                        tb1

                )
        );

        rbga.stop_drive();


        sleep(500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
        rbga.slide(rbga.arm_slide_extend);
        sleep(1500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip1);
        rbga.slide(rbga.arm_slide_extend-200);
        sleep(500);



    }

    void turn1back(){
        Actions.runBlocking(
                new SequentialAction(
                        turn1back
                )
        );

        rbga.stop_drive();

    }
    void bturn1back(){
        Actions.runBlocking(
                new SequentialAction(
                        bturn1back
                )
        );

        rbga.stop_drive();

    }









 void autoroute2() {
     Actions.runBlocking(
             new SequentialAction(
                     tr2

             )
     );

     rbga.stop_drive();


     sleep(500);
     rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
     rbga.slide(rbga.arm_slide_extend);
     sleep(1500);
     rbga.arm_handle.setPosition(rbga.arm_handle_ip1);



    }

    void bautoroute2() {
        Actions.runBlocking(
                new SequentialAction(
                        tb2

                )
        );

        rbga.stop_drive();


        sleep(500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
        rbga.slide(rbga.arm_slide_extend);
        sleep(1500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip1);



    }

    void autoroute3() {
        telemetry.addLine("in, this part");
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        tr3

                )
        );

        rbga.stop_drive();


        sleep(500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
        rbga.slide(rbga.arm_slide_extend);
        sleep(1500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip1);
        rbga.slide(rbga.arm_slide_extend-200);
        sleep(500);



    }

    void bautoroute3() {
        Actions.runBlocking(
                new SequentialAction(
                        tb3

                )
        );

        rbga.stop_drive();


        sleep(500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip0);
        rbga.slide(rbga.arm_slide_extend);
        sleep(1500);
        rbga.arm_handle.setPosition(rbga.arm_handle_ip1);
        rbga.slide(rbga.arm_slide_extend-200);
        sleep(500);



    }

 }





