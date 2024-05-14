//package org.firstinspires.ftc.teamcode;
//
////import com.qualcomm.hardware.lynx.LynxModule;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@TeleOp(name = "CenterstageTele", group = "A")
//
//
//
//
//
//public class ACenterStage extends LinearOpMode {
//
//    protected DcMotorEx front_left;
//
//    protected DcMotorEx front_right;
//    protected DcMotorEx rear_left;
//    protected DcMotorEx rear_right;
//
//    protected DcMotorEx arm_rotate;
//    protected DcMotorEx arm_slide;
//
//    protected Servo arm_grab;
//    protected Servo arm_handle;
//
//
//
//
//
//    boolean robo_drive = true;
//    boolean end_flag = false;
//
//    public void ZeroSlide(){
//        arm_slide.setTargetPosition(0);
//        arm_slide.setVelocity(1000);
//    }
//
//
//    public void arm_rot(int targ, int vel){
//        arm_rotate.setTargetPosition(targ);
//        arm_rotate.setVelocity(vel);
//    }
//
//    public void arm_slider(int targ, int vel){
//        arm_slide.setTargetPosition(targ);
//        arm_slide.setVelocity(vel);
//    }
//
//
//
//    //extend linear slide all the way out
//    public void extend_slide(){
//        arm_slider(0,1000);
//    }
//
//    // grab pixels and return to drive state for the armn
//    public void in_grab(){
//
//        arm_rot(20,2500);
//        arm_handle.setPosition(0);
//        sleep(500);
//        arm_grab.setPosition(0.275);
//        sleep(1000);
//        arm_rot(100,2500);
//        arm_slider(-2200,1500);
//
//    }
//
//    public void out_ready() {
//        arm_rot(900,1500);
////        arm_handle.setPosition(0.75);
//        sleep(5000);
//        extend_slide();
//    }
//
//    public void out_release(){
//
//        arm_grab.setPosition(0.245);
//        sleep(5000);
//        arm_grab.setPosition(0.225);
//        sleep(5000);
////        arm_rot(100,500);
////        arm_slider(-1600,1000);
////        arm_handle.setPosition(0);
//    }
//
//
//
//
//
//
////    double loop_time = 0;
////    double loopi = 0;
////    double average_loop;
//
//
//
//
//    @Override
//    public void runOpMode() {
//
//        front_left = hardwareMap.get(DcMotorEx.class, "front_left");
//        front_right = hardwareMap.get(DcMotorEx.class, "front_right");
//        rear_left = hardwareMap.get(DcMotorEx.class, "rear_left");
//        rear_right = hardwareMap.get(DcMotorEx.class, "rear_right");
//
//        arm_rotate = hardwareMap.get(DcMotorEx.class, "arm_rotate");
//        arm_slide = hardwareMap.get(DcMotorEx.class, "arm_slide");
//        arm_grab = hardwareMap.get(Servo.class, "arm_grab");
//        arm_handle = hardwareMap.get(Servo.class, "arm_handle");
//        arm_rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm_rotate.setTargetPosition(0);
//        arm_rotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        arm_rotate.setVelocity(0);
//
//    arm_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm_slide.setTargetPosition(0);
//        arm_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        arm_slide.setVelocity(0);
//
//
//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        waitForStart();
//
//
////        arm_grab.setPosition(0);
//        arm_handle.setPosition(0);
//
//        arm_rotate.setTargetPosition(100);
//        arm_rotate.setVelocity(2500);
//
//        arm_slide.setTargetPosition(-2200);
//        arm_slide.setVelocity(1300);
//
//
//
//
//
//
//        int arm_pos = 50;
//        int slide_pos = 0;
//
//        double handle_pos = 0.0;
//        double arm_grab_pos = 0.0;
//        boolean prev_bump_right = false;
//        boolean prev_bump_left = false;
//
//        boolean prev_dpad_up = false;
//        boolean prev_dpad_down = false;
//
//
//
//
//
//
//
//
//
//        while (opModeIsActive()) {
//
//
//
//            double y = -gamepad1.right_stick_y;
//            double x = gamepad1.right_stick_x * 1.1;
//            double rx = gamepad1.left_stick_x*0.75;
//
//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;
//
//            front_left.setPower(-frontLeftPower);
//            rear_left.setPower(-backLeftPower);
//            front_right.setPower(frontRightPower);
//            rear_right.setPower(backRightPower);
//
//            if (gamepad1.cross){
//                ZeroSlide();
//            }
//            //extend slide
//            if (gamepad2.dpad_up){
//                extend_slide();
//            }
//
//            if (gamepad2.circle){
//                in_grab();
//            }
//
//            if (gamepad2.dpad_down){
//                out_ready();
//            }
//            if(gamepad2.square){
//                out_release();
//            }
//
//            if (gamepad2.dpad_left && arm_pos>=10){
//                arm_pos-=5;
//                arm_rot(arm_pos,300);
//            }
//
//            if (gamepad2.dpad_right&&arm_pos<=150){
//                arm_pos+=5;
//                arm_rot(arm_pos,300);
//            }
//
//            telemetry.addData("arm_pos ", arm_pos);
//            telemetry.update();
//
//
//
//
//
//
//
//
//
//
//
//        }
//    }
//}
//
//
//
//
//
//
//
//
//
