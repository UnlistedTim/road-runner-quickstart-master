package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.IMU;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


public class Baseauto extends BaseClass {

    int innum = 3;

    public Baseauto(LinearOpMode op, Pose2d p1) {
        super(op, p1);
    }

    public ElapsedTime runtimea = new ElapsedTime();

    //Pose2d a1=new Pose2d ();





    void route_setup()

    {

        Action tr1;
        Action tr2;
        Action tr3;
        Pose2d start=new Pose2d(0,0,Math.toRadians(0));
        Pose2d r1= new Pose2d(0,5,Math.toRadians(45));
        Pose2d r2= new Pose2d(0,10,Math.toRadians(0));
        Pose2d r3= new Pose2d(0,5,Math.toRadians(-45));

        MecanumDrive drive = new MecanumDrive(super.Op.hardwareMap, start);

        tr1 = drive.actionBuilder(start)

                .splineToLinearHeading(r1,0)
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
    }










    public void afinalxy() {
        drop_qty = 2;


        if (index == 1) {
            input1x = 5;
            input1y = cycle + 1;
            //if(cycle==2) input1x=6;
            finalxy();

        }  //target y=2,x=5;

        else {


            input1x = 3;
            input1y = cycle + 1;
            // if(!baseblue) {input1x=2;}
            finalxy();


        }


        if (cycle == 2) finaly = finaly + layer_rate + 50;


    }

    public void coordinate_input(boolean tri, boolean cir, boolean cross,boolean square)
    {
//
        if(!tri&& !cir&& !cross&&!square) return;
        if(tri) {
            innum=1 ;
        }
        else if (cir) {
            innum=2;
        }
        else if (cross) {
            innum=3;
        }
        else if (square) {
            innum=4;
        }
        else return;
        timerinput(0);

    }



    public void drone_fly(){
        drone.setPosition(drone_launch);
        pause(500);
        drone.setPosition(drone_idle);
    }

    public void coordinate_confirm(boolean touch){
        if (!touch) return;
        if (touch){
            tempinput = innum;
        }
    }


    public void slide(int target){

        if (target >= -5300 && target <= 0){
            arm_slide.setTargetPosition(target);
            arm_slide.setVelocity(slide_vel);
        }

    }





    public void slide_extend(){
        slide(0);

    }

    public void intake_grab(){


        arm_rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





        arm_handle.setPosition(arm_handle_ip1);
        pause(300);
//        arm_grab.setPosition(arm_grab_hold);
        bottom_claw.setPosition(bottom_claw_hold);
        top_claw.setPosition(top_claw_hold);
        arm_slide.setTargetPosition(0);
        arm_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pause(600);
        arm_handle.setPosition(arm_handle_idle);
        pause(100);


        slide(arm_slide_idle);

        arm_rotate.setTargetPosition(0);
        arm_rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void outtake_turn(){
//        rotate(arm_rotate_op7);

        rotate(arm_rotate_op4 - 200);
        arm_handle.setPosition(arm_handle_op4);
//        if (tempinput == 1){
//            rotate(arm_rotate_op1);
//            arm_handle.setPosition(arm_handle_op1);
//        }
//        else if (tempinput == 2){
//            rotate(arm_rotate_op2);
//            arm_handle.setPosition(arm_handle_op2);
//        }
//        else if (tempinput == 3){
//            rotate(arm_rotate_op3);
//            arm_handle.setPosition(arm_handle_op3);
//        }
//        else if (tempinput == 4){
//            rotate(arm_rotate_op4);
//            arm_handle.setPosition(arm_handle_op4);
//        }

    }


    public void outtake_1release(){
        bottom_claw.setPosition(bottom_claw_idle);

        pause(500);


    }

    public void outtake_ready(){
        rotate(arm_rotate_op1);
        if (tempinput == 1){
            slide(arm_slide_op1);
        }
        else if (tempinput == 2){
            slide(arm_slide_op2);

        }
        else if (tempinput == 3){
            slide(arm_slide_op3);

        }
        else if (tempinput == 4){
            slide(arm_slide_op4);

        }
        else if (tempinput == 5){
            slide(arm_slide_op5);

        }
        else if (tempinput == 6){
            slide(arm_slide_op6);
        }


    }

    public void manual_slide_up(){

        if (tempinput < 6){
            tempinput++;
        }
        outtake_ready();
    }

    public void manual_slide_down(){
        if (tempinput > 0){
            tempinput--;
        }
        outtake_ready();
    }

    public void outtake_2release(){

        top_claw.setPosition(top_claw_idle+0.03);

        //arm_grab.setPosition(arm_grab_idle);
        pause(200);
        rotate(arm_rotate_out_buffer);
        slide(arm_slide_idle);
        arm_handle.setPosition(arm_handle_idle);
        //pause(500);
        rotate(arm_rotate_buffer);
    }



    public void rotate(int target){
        if (target >=0 && target <= 2200){
            arm_rotate.setTargetPosition(target);
            arm_rotate.setVelocity(arm_vel);
        }
    }

    public void intake_ready(){
        rotate(arm_rotate_ground); //arm_rotate_ground
        slide(arm_slide_extend);
        arm_handle.setPosition(arm_handle_ip0);
//        arm_grab.setPosition(arm_grab_idle);
        bottom_claw.setPosition(bottom_claw_open);
        top_claw.setPosition(top_claw_open);
        arm_rotate.setPower(0);
    }

    public void hang_ready(){
        stop_drive();
        slide(arm_slide_idle);
        pause(500);
        rotate(arm_rotate_hang);
        pause(500);
        slide(arm_slide_extend);


    }
    public void hang(){
        stop_drive();
        slide(arm_slide_hang);
        pause(100000);

    }


    public void outtakeauto(){
        stop_drive();

        pause(500);

        rotate( arm_rotate_op1);
        pause(500);
        timer3(0);

        while(Op.opModeIsActive() &&  gap >=9 && ! timer3(3000)){
            dist_align();
//            telemetry.addData("front_dist",  front_dist.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
        stop_drive();
        pause(500);

        // outake 2 release
        top_claw.setPosition( top_claw_idle+0.04);

        //arm_grab.setPosition(arm_grab_idle);
        pause(500);
        rotate( arm_rotate_out_buffer);
        slide( arm_slide_idle);
        arm_handle.setPosition( arm_handle_idle);
        //pause(500);
        rotate( arm_rotate_buffer);
        pause(1000);
        rotate( arm_rotate_ground);
        slide( arm_slide_extend);
        pause(50000);
    }

    public void autoback(){

        timer2(0);

        while (Op.opModeIsActive() && !timer2(3000)){
             targetFound = false;
             desiredTag  = null;

            // Step through the list of detected tags arnd look for a matching tag
            List<AprilTagDetection> currentDetections =  aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ( detection.id ==  DESIRED_TAG_ID) {
                        // Yes, we want to use this tag.
                         targetFound = true;
                         desiredTag = detection;
//                        telemetry.addData("Detection ID", detection.id);
                        break;  // don't look any further.

                    }
                }
            }

            if ( targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = ( front_dist.getDistance(DistanceUnit.INCH) -  DESIRED_DISTANCE_T);
                double  headingError    =  base_align_angle -  imu.getRobotYawPitchRollAngles().getYaw((AngleUnit.DEGREES));
                double yawError =  desiredTag.ftcPose.yaw -  STRAFE_TARG;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                 drive  = Range.clip(rangeError *  AUTOSPEED_GAIN, - MAX_AUTO_SPEED,  MAX_AUTO_SPEED);
                 turn   = -Range.clip(headingError *  TURN_GAIN, - MAX_AUTO_TURN,  MAX_AUTO_TURN) ;
                 strafe = -Range.clip(yawError *  STRAFE_GAIN, - MAX_AUTO_STRAFE,  MAX_AUTO_STRAFE);
                 moveRobot( drive,  strafe,  turn,1, true);

//                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ",  drive,  strafe,  turn);
//                telemetry.addData("Range", "%5.1f inches",  desiredTag.ftcPose.range);
//                telemetry.addData("Bearing", "%3.0f degrees",  desiredTag.ftcPose.bearing);
//                telemetry.addData("Yaw", "%3.0f degrees",  desiredTag.ftcPose.yaw);
            }
            else {
                 stop_drive();
            }

        }

         stop_drive();

        stop_drive();

        pause(500);

         rotate( arm_rotate_op1);
        pause(500);
         timer3(0);

        while(Op.opModeIsActive() &&  gap >=16 && ! timer3(3000)){
             dist_align();
//            telemetry.addData("front_dist",  front_dist.getDistance(DistanceUnit.MM));
//            telemetry.update();
        }
         stop_drive();
        pause(500);

        // outake 2 release
         top_claw.setPosition( top_claw_idle+0.03);

        //arm_grab.setPosition(arm_grab_idle);
        pause(500);
         rotate( arm_rotate_out_buffer);
         slide( arm_slide_idle);
         arm_handle.setPosition( arm_handle_idle);
        //pause(500);
         rotate( arm_rotate_buffer);
        pause(1000);
         rotate( arm_rotate_ground);
         slide( arm_slide_extend);
        pause(50000);


    }









    void bfinalxy() {
        if ((!baseblue && wall_route) || (baseblue && !wall_route)) {
            if (cycle == 0) {
                input1x = 1;
                input1y = 1;
                input2x = input1x + 3;
                input2y = input1y;
                if (index == 3) {
                    input1x = 3;
                    input2x = input1x - 2;
                }
            } else {
                if (index < 3) input1x = 4;
                else input1x = 1;
                input1y = cycle + 1;
                input2y = input1y;
            }
        } else {
            if (cycle == 0) {
                input1x = 6;
                input1y = 1;
                input2x = input1x - 3;
                input2y = input1y;
                if (index == 1) {
                    input1x = 4;
                    input2x = input1x + 2;
                }
            } else {
                if (index > 1) input1x = 4;
                else input1x = 6;
                if (cycle == 1) input1y = 3;
                if (cycle == 2 && index == 1) {
                    input1y = 4;
                    input1x = input1x + 1;
                }
            }

        }

        finalxy();
    }








    public void buildTrajectories() {
// middleroute

        Pose2d r_r_1_1 = new Pose2d(23.5, 6, Math.toRadians(50));//44
        Pose2d r_r_2_1 = new Pose2d(28, -3, Math.toRadians(0));
        Pose2d r_r_3_1 = new Pose2d(19.5, -9.5, Math.toRadians(-1));
        Pose2d r_r_0_2 = new Pose2d(25, -32, Math.toRadians(90));//26


        Pose2d b_l_1_1 = new Pose2d(19, 8, Math.toRadians(0));
        Pose2d b_l_2_1 = new Pose2d(26, 1.5, Math.toRadians(0));
        Pose2d b_l_3_1 = new Pose2d(23, -7.5, Math.toRadians(-46));
        Pose2d b_l_0_2 = new Pose2d(27.5, 29, Math.toRadians(-89));


        Pose2d r_l_1_1 = new Pose2d(28, 8, Math.toRadians(0));
        Pose2d r_l_2_1 = new Pose2d(29, 2.5, Math.toRadians(0));
        Pose2d r_l_3_1 = new Pose2d(25, -9, Math.toRadians(-46));
        Pose2d r_l_0_2 = new Pose2d(24.5, 13, Math.toRadians(92));//25


        Pose2d b_r_1_1 = new Pose2d(22.5, 6.5, Math.toRadians(50));//44
        Pose2d b_r_2_1 = new Pose2d(29, -3.0, Math.toRadians(0));
        Pose2d b_r_3_1 = new Pose2d(28.5, -11, Math.toRadians(0));
        Pose2d b_r_0_2 = new Pose2d(25.5, -13, Math.toRadians(-94));//26
        Pose2d b_r_1_2 = new Pose2d(25, -13, Math.toRadians(-96));//26


    }
}
