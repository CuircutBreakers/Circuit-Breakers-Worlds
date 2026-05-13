package org.firstinspires.ftc.teamcode.Auto;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Autonomous
public class BlueBack3Ball extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intake;
    private DcMotor spindle;
    private DcMotorEx launcherLeft, launcherRight;
    private HuskyLens camera;

    HuskyLens Camera;

    private static final int SPINDLE_OPEN   = -130;
    private static final int SPINDLE_1BALL  = -225;
    private static final int SPINDLE_2BALL  = -265;

    // Launch spindle positions (encoder ticks) - TUNE THESE
    private static final int SPINDLE_LAUNCH_1 = -350;
    private static final int SPINDLE_LAUNCH_2 = -540;
    private static final int SPINDLE_LAUNCH_3 = -650;

    private static final double SPINDLE_POWER = 1;




    // Over-voltage reduction (tune)
    // Voltage compensation (tune)
// Target band ~13.8V (typical "good" under load)

    @Override
    public void runOpMode() throws InterruptedException {

        //region init
        MecanumDrive.Params PARAMS = new MecanumDrive.Params();

        MecanumKinematics kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

        MinVelConstraint IntakeSpeed =
                new MinVelConstraint(Arrays.asList(kinematics.new WheelVelConstraint(5), new AngularVelConstraint(PARAMS.maxAngVel)));




        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-64.25 ,16,Math.toRadians(0)));
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        // Reverse left side drivetrain
        spindle = hardwareMap.get(DcMotor.class, "spindle");
        spindle.setTargetPosition(0);
        spindle.setPower(SPINDLE_POWER);
        spindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        camera = hardwareMap.get(HuskyLens.class, "huskylens");
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        //endregion

        waitForStart();
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-64.25 ,16,Math.toRadians(0)))
                        .stopAndAdd(new Intake(intake,.4))
                        .strafeToLinearHeading(new Vector2d(-35,10),Math.toRadians(0))
                        .waitSeconds(.75)
                        .stopAndAdd(new PrimeLaunchers(launcherLeft,launcherRight,725))
                        .strafeToLinearHeading(new Vector2d(15,10),Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(30.5,20),Math.toRadians(55))
                        .stopAndAdd(new Launch(spindle,1))
                        .waitSeconds(1)
                        .stopAndAdd(new Launch(spindle,2))
                        .waitSeconds(1)
                        .stopAndAdd(new Launch(spindle,3))
                        .waitSeconds(.5)
                        .afterTime(0,new Spindle(spindle,1))
                        .stopAndAdd(new PrimeLaunchers(launcherLeft,launcherRight,0))
                        .strafeToLinearHeading(new Vector2d(20,8),Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-20,8),Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(-36,28),Math.toRadians(90))
                        .afterDisp(0, new Intake(intake,1))
                        .strafeToLinearHeading(new Vector2d(-36,32),Math.toRadians(90),IntakeSpeed)
                        .afterDisp(0,new Spindle(spindle,2))
                        .waitSeconds(.65)
                        .afterDisp(0, new Intake(intake,.5))
                        .strafeToLinearHeading(new Vector2d(-36,38),Math.toRadians(90),IntakeSpeed)
                        .afterDisp(0,new Spindle(spindle,3))
                        .waitSeconds(.5)
                        .strafeToLinearHeading(new Vector2d(-36,43),Math.toRadians(90),IntakeSpeed)
                        .afterDisp(0, new Intake(intake, .3))
                        .stopAndAdd(new PrimeLaunchers(launcherLeft,launcherRight,775))
                        .strafeToLinearHeading(new Vector2d(14,14),Math.toRadians(45))
                        .stopAndAdd(new Launch(spindle,1))
                        .waitSeconds(1)
                        .stopAndAdd(new Launch(spindle,2))
                        .waitSeconds(1)
                        .stopAndAdd(new Launch(spindle,3))
                        .waitSeconds(.5)
                        .afterTime(0,new Spindle(spindle,1))
                        .strafeToLinearHeading(new Vector2d(-20,20),Math.toRadians(180))
                        .build()
        );
    }

    public static class Intake implements Action {
        DcMotor intake;
        double power;
        public Intake(DcMotor intake, double Power) {
            this.intake = intake;
            this.power = Power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setPower(power);
            return false;
        }
    }

    public static class Spindle implements Action {
        DcMotor spindle;
        double position;
        public Spindle(DcMotor spindle, double pos) {
            this.spindle = spindle;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(position == 1){
                spindle.setTargetPosition(130);
            }
            if (position == 2){
                spindle.setTargetPosition(50);
            }
            if (position == 3){
                spindle.setTargetPosition(10);
            }
            return false;
        }
    }

    public static class Launch implements Action {
        DcMotor spindle;
        double position;
        public Launch(DcMotor spindle, double pos) {
            this.spindle = spindle;
            this.position = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(position == 1){
                spindle.setTargetPosition(-75);
            }
            if (position == 2){
                spindle.setTargetPosition(-250);
            }
            if (position == 3){
                spindle.setTargetPosition(-340);
            }
            return false;
        }
    }

    public static class PrimeLaunchers implements Action {
        DcMotorEx LaunchLeft;
        DcMotorEx LaunchRight;
        double Power;
        public PrimeLaunchers(DcMotorEx MotorLeft,DcMotorEx MotorRight, double power)
        {
            this.LaunchLeft = MotorLeft;
            this.LaunchRight =MotorRight;
            this.Power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            LaunchLeft.setVelocity(-Power);
            LaunchRight.setVelocity(Power);
            return false;
        }
    }



    public static class TagResult {
        public volatile int id = 0;          // 0 = not found yet
        public volatile boolean found = false;
    }

    public static class DetectAprilTag implements Action {

        private final HuskyLens camera;
        private final TagResult out;
        private final double timeoutSec;

        private ElapsedTime timer;

        public DetectAprilTag(HuskyLens camera, TagResult out, double timeoutSec) {
            this.camera = camera;
            this.out = out;
            this.timeoutSec = timeoutSec;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            if (timer == null) timer = new ElapsedTime();

            HuskyLens.Block[] blocks = camera.blocks();

            for (HuskyLens.Block b : blocks) {

                // Only accept IDs 1–3
                if (b.id >= 1 && b.id <= 3) {
                    out.id = b.id;
                    out.found = true;

                    packet.put("tag/found", true);
                    packet.put("tag/id", out.id);

                    return false;
                }
            }

            // Optional timeout fallback
            if (timer.seconds() >= timeoutSec) {
                out.id = 1;          // default fallback
                out.found = false;

                packet.put("tag/timeout", true);
                return false;
            }

            packet.put("tag/found", false);
            return true; // keep checking
        }
    }




}