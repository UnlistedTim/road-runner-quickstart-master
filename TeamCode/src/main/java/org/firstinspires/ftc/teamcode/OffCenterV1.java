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
////import org.firstinspires.ftc.teamcode.PIDF_Arm;
//
//
//@TeleOp(name = "CenterDebug", group = "A")
//
//
//
//
//
//public class OffCenterV1 extends LinearOpMode {
//
//    protected DcMotorEx front_left;
//
////    protected PIDController pidf = new PIDController(PIDF_Arm.p,PIDF_Arm.i,PIDF_Arm.d);
////
////    public double pidf_output(int current_pos, int target_pos){
////        return pidf.calculate(
////                current_pos, target_pos);
////    }
//
//
//
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
//        arm_grab.scaleRange(0.44,0.84);
//
//
//    arm_slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm_slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm_slide.setTargetPosition(0);
//        arm_slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        arm_slide.setVelocity(0);
//
//        arm_handle.setDirection(Servo.Direction.REVERSE);
//
//        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rear_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rear_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        waitForStart();
//
//
////        arm_grab.setPosition(0.15);
////        arm_handle.setPosition(0.03);
//
//        arm_grab.setPosition(0.4);
//        arm_handle.setPosition(0.24);
//
//
//
//
//
//
//        int arm_pos = 0;
//        int slide_pos = 0;
//
//        double handle_pos = 0.24;
//        double arm_grab_pos = 0.4;
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
//        while (opModeIsActive()) {
//
////            double ff = Math.cos(Math.toRadians(arm_rotate.getCurrentPosition()/PIDF_Arm.ticks_in_degree)) * PIDF_Arm.f;
////            arm_rotate.setPower( pidf_output(arm_rotate.getCurrentPosition(),arm_pos) + ff);
//
//
//
//
//
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
//
//            //Arm rotate debug code
//
//            if (gamepad2.right_bumper && arm_pos<3000){
//                sleep(400);
//                arm_pos+=100;
//                arm_rotate.setTargetPosition(arm_pos);
//                arm_rotate.setVelocity(1600);
//
//            }
//
//            if(gamepad2.left_bumper && arm_pos>0){
//                sleep(400);
//                arm_pos-=100;
//                arm_rotate.setTargetPosition(arm_pos);
//                arm_rotate.setVelocity(1600);
//
//            }
//
//
//            // Arm slide debug code
//
//            if(gamepad2.circle && slide_pos < 0) {
//                sleep(400);
//                slide_pos += 200;
//                arm_slide.setTargetPosition(slide_pos);
//                arm_slide.setVelocity(1500);
//            }
//
//            if(gamepad2.square && slide_pos > -2600 ){
//                sleep(400);
//                slide_pos-=200;
//                arm_slide.setTargetPosition(slide_pos);
//                arm_slide.setVelocity(1500);
//            }
//
//
//
//
//            // Arm grabber claw debug code
//            if (gamepad2.dpad_up && arm_grab_pos<1.0){
//                sleep(400);
//                arm_grab_pos  +=0.02;
//                arm_grab.setPosition(arm_grab_pos);
//            }
//
//            if(gamepad2.dpad_down && arm_grab_pos>0){
//                sleep(400);
//                arm_grab_pos-=0.02;
//                arm_grab.setPosition(arm_grab_pos);
//            }
//
//            //arm handler debug code
//
//
//            if (gamepad2.dpad_right && handle_pos<0.76){
//                sleep(400);
//                handle_pos  +=0.01;
//                arm_handle.setPosition(handle_pos);
//            }
//
//            if(gamepad2.dpad_left && handle_pos>0.1){
//                sleep(400);
//                handle_pos-=0.01;
//                arm_handle.setPosition(handle_pos);
//            }
//
//            //Both lock
//            if (gamepad1.circle){
//                arm_grab.setPosition(0.6);
//                sleep(500);
//            }
//            //First pixel release
//            if (gamepad1.cross){
//                arm_grab.setPosition(0.53);
//                sleep(500);
//            }
//
//
//            //second Pixel release
//            if (gamepad1.square){
//                arm_grab.setPosition(0.15);
//                sleep(500);
//
//            }
//
//            telemetry.addData("Arm_rotate Gunner.right/left bumper", arm_pos );
//            telemetry.addData("Arm_slide Gunner.circle/square", slide_pos);
//            telemetry.addData("Arm_grab Gunner.dpad_up/dpad_down", arm_grab_pos);
//            telemetry.addData("Arm_handler Gunner.dpad_right,dpad_left", handle_pos);
//            telemetry.update();
//
////            if (gamepad2.dpad_righ
////            t && handle_pos <= 0.95){
////                sleep(500);
////                handle_pos+=0.05;
////                arm_handle.setPosition(handle_pos);
////            }
////            if (gamepad2.dpad_left && handle_pos>=0.05){
////                sleep(500);
////                handle_pos-=0.05;
////                arm_handle.setPosition(handle_pos);
////            }
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
