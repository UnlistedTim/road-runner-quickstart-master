package org.firstinspires.ftc.teamcode;
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
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;


public class Baseauto extends BaseClass {

    public Baseauto(LinearOpMode op, Pose2d p1) {
        super(op, p1);
    }

    public ElapsedTime runtimea = new ElapsedTime();










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


    public void slide(int target){

        if (target >= -4250 && target <= 0){
            arm_slide.setTargetPosition(target);
            arm_slide.setVelocity(slide_vel);
        }

    }

    public void slide_extend(){
        slide(0);


    }

    public void intake_grab(){
        arm_handle.setPosition(arm_handle_ip1);
        pause(500);
        arm_grab.setPosition(arm_grab_hold);
        pause(500);
        arm_handle.setPosition(arm_handle_op7);
        slide(arm_slide_idle);

    }

    public void outtake_turn(){
        rotate(arm_rotate_op7);
        pause(1500);
        slide_extend();
    }

    public void outtake_1release(){
        arm_grab.setPosition(arm_grab_open2);
        pause(500);

    }

    public void outtake_2release(){

        arm_grab.setPosition(arm_grab_idle);
        pause(500);
        rotate(arm_rotate_op7-150);
        arm_handle.setPosition(arm_handle_idle);


        slide(arm_slide_idle);
        pause(1000);
        rotate(100);
    }



    public void rotate(int target){
        if (target >=0 && target <= 2200){
            arm_rotate.setTargetPosition(target);
            arm_rotate.setVelocity(arm_vel);
        }
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
