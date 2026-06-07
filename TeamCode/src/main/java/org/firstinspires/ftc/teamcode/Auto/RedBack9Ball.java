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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Teleop.PoseStorage;

import java.util.Arrays;

@Autonomous
public class RedBack9Ball extends LinearOpMode {
    private static final int SPINDLE_OPEN   = 150;
    private static final int SPINDLE_1BALL  = 43;
    private static final int SPINDLE_2BALL  = 22;

    private static final int SPINDLE_LAUNCH_1 = -65;
    private static final int SPINDLE_LAUNCH_2 = -240;
    private static final int SPINDLE_LAUNCH_3 = -330;

    private static final double SPINDLE_POWER = 1.0;



    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive.Params PARAMS = new MecanumDrive.Params();

        MecanumKinematics kinematics = new MecanumKinematics(
                PARAMS.inPerTick * PARAMS.trackWidthTicks,
                PARAMS.inPerTick / PARAMS.lateralInPerTick
        );

        MinVelConstraint IntakeSpeed =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(8),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));
        MinVelConstraint IntakeSpeedFast =
                new MinVelConstraint(Arrays.asList(
                        kinematics.new WheelVelConstraint(18),
                        new AngularVelConstraint(PARAMS.maxAngVel)
                ));

        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                new Pose2d(-64.25, -16, Math.toRadians(0))
        );

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        DcMotorEx launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        DcMotorEx launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        launcherLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");

        spindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindle.setTargetPosition(SPINDLE_2BALL);
        spindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindle.setPower(1);
        sleep(500);
        spindle.setPower(0);

        TouchSensor intakeTouch = hardwareMap.get(TouchSensor.class, "TouchSensor");

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-64.25, -16, Math.toRadians(0)))

                        .stopAndAdd(new PrimeLaunchers(launcherLeft, launcherRight, 2650))
                        .stopAndAdd(new Intake(intake, 0.4))
                        .strafeToLinearHeading(new Vector2d(-55, -16), Math.toRadians(-20))
                        .waitSeconds(.75)

                        .stopAndAdd(new Launch(spindle, 1))
                        .waitSeconds(.5)
                        .stopAndAdd(new Launch(spindle, 2))
                        .waitSeconds(.5)
                        .stopAndAdd(new Launch(spindle, 3))
                        .waitSeconds(0.5)

                        .afterTime(0, new Spindle(spindle, 1))
                        .stopAndAdd(new PrimeLaunchers(launcherLeft, launcherRight, 0))

                        //Back spike mark
                        .strafeToLinearHeading(new Vector2d(-41, -23), Math.toRadians(-90))

                        .afterDisp(0, new AutoIntake(
                                intake,
                                spindle,
                                intakeTouch,
                                2,
                                .75,
                                .75,
                                4.0
                        ))
                        .strafeToLinearHeading(new Vector2d(-41, -62), Math.toRadians(-90))
                        .afterTime(0, new Intake(intake,.8))
                        .afterTime(.5,new PrimeLaunchers(launcherLeft, launcherRight, 2650))
                        .strafeToLinearHeading(new Vector2d(-55, -16), Math.toRadians(-20))

                        .stopAndAdd(new Launch(spindle, 1))
                        .waitSeconds(.5)
                        .stopAndAdd(new Launch(spindle, 2))
                        .waitSeconds(0.5)
                        .stopAndAdd(new Launch(spindle, 3))
                        .waitSeconds(.5)

                        .afterTime(0, new Spindle(spindle, 1))
                        .stopAndAdd(new PrimeLaunchers(launcherLeft, launcherRight, 0))

                        // Human player spike mark-------------------


                        .strafeToLinearHeading(new Vector2d(-61, -54.5), Math.toRadians(-100))

                        .afterDisp(0, new AutoIntake(
                                intake,
                                spindle,
                                intakeTouch,
                                2,
                                .75,
                                .75,
                                4.0
                        ))
                        .waitSeconds(.25)
                        .afterTime(0, new Spindle(spindle,2))
                        .strafeToLinearHeading(new Vector2d(-72, -54.5), Math.toRadians(-100))
                        .strafeToLinearHeading(new Vector2d(-75, -58), Math.toRadians(-160))
                        .afterTime(0, new Intake(intake,.8))
                        .afterTime(0,new PrimeLaunchers(launcherLeft, launcherRight, 2650))
                        .strafeToLinearHeading(new Vector2d(-55, -16), Math.toRadians(-23))

                        .stopAndAdd(new Launch(spindle, 1))
                        .waitSeconds(.5)
                        .stopAndAdd(new Launch(spindle, 2))
                        .waitSeconds(.5)
                        .stopAndAdd(new Launch(spindle, 3))
                        .waitSeconds(.5)

                        .afterTime(0, new Spindle(spindle, 1))
                        .stopAndAdd(new PrimeLaunchers(launcherLeft, launcherRight, 0))


                        .strafeToLinearHeading(new Vector2d(-45, -25), Math.toRadians(0))
                        .build()
        );
        drive.updatePoseEstimate();
        PoseStorage.currentPose = drive.localizer.getPose();
    }

    public static class AutoIntake implements Action {
        private final DcMotor intake;
        private final DcMotor spindle;
        private final TouchSensor touchSensor;

        private final int targetCount;
        private final double intakePower;
        private final double holdPower;
        private final double timeoutSeconds;

        private static final double ARTIFACT_COUNT_COOLDOWN = 0.10;
        private static final double SPINDLE_DELAY_SECONDS = 0.05; // 50 ms

        private final ElapsedTime timer = new ElapsedTime();
        private final ElapsedTime countCooldownTimer = new ElapsedTime();
        private final ElapsedTime spindleDelayTimer = new ElapsedTime();

        private boolean started = false;
        private boolean artifactWasDetected = false;
        private boolean spindleMovePending = false;

        private int artifactCount = 0;
        private int pendingSpindleTarget = 0;

        public AutoIntake(
                DcMotor intake,
                DcMotor spindle,
                TouchSensor touchSensor,
                int targetCount,
                double intakePower,
                double holdPower,
                double timeoutSeconds
        ) {
            this.intake = intake;
            this.spindle = spindle;
            this.touchSensor = touchSensor;
            this.targetCount = targetCount;
            this.intakePower = intakePower;
            this.holdPower = holdPower;
            this.timeoutSeconds = timeoutSeconds;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!started) {
                started = true;

                timer.reset();
                countCooldownTimer.reset();
                spindleDelayTimer.reset();

                artifactWasDetected = false;
                spindleMovePending = false;

                artifactCount = 0;
                pendingSpindleTarget = 0;

                intake.setPower(intakePower);
            }

            boolean detected = touchSensor.isPressed();

            boolean allowedToCount =
                    countCooldownTimer.seconds() >= ARTIFACT_COUNT_COOLDOWN;

            if (detected && !artifactWasDetected && allowedToCount) {
                artifactCount++;
                countCooldownTimer.reset();

                if (artifactCount == 1) {
                    pendingSpindleTarget = SPINDLE_1BALL;
                } else if (artifactCount >= 2) {
                    pendingSpindleTarget = SPINDLE_2BALL;
                }

                spindleDelayTimer.reset();
                spindleMovePending = true;
            }

            if (spindleMovePending &&
                    spindleDelayTimer.seconds() >= SPINDLE_DELAY_SECONDS) {

                spindle.setTargetPosition(pendingSpindleTarget);
                spindle.setPower(SPINDLE_POWER);

                spindleMovePending = false;
            }

            artifactWasDetected = detected;

            telemetryPacket.put("Auto Intake Count", artifactCount);
            telemetryPacket.put("Touch Pressed", detected);
            telemetryPacket.put("Auto Intake Time", timer.seconds());
            telemetryPacket.put("Count Cooldown", countCooldownTimer.seconds());
            telemetryPacket.put("Spindle Pending", spindleMovePending);
            telemetryPacket.put("Pending Spindle Target", pendingSpindleTarget);

            if (artifactCount >= targetCount || timer.seconds() >= timeoutSeconds) {
                intake.setPower(holdPower);
                return false;
            }

            return true;
        }
    }
    public static class Intake implements Action {
        DcMotor intake;
        double power;

        public Intake(DcMotor intake, double power) {
            this.intake = intake;
            this.power = power;
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
            if (position == 1) {
                spindle.setTargetPosition(SPINDLE_OPEN);
            }
            if (position == 2) {
                spindle.setTargetPosition(SPINDLE_1BALL);
            }
            if (position == 3) {
                spindle.setTargetPosition(SPINDLE_2BALL);
            }

            spindle.setPower(SPINDLE_POWER);
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
            if (position == 1) {
                spindle.setTargetPosition(SPINDLE_LAUNCH_1);
            }
            if (position == 2) {
                spindle.setTargetPosition(SPINDLE_LAUNCH_2);
            }
            if (position == 3) {
                spindle.setTargetPosition(SPINDLE_LAUNCH_3);
            }

            spindle.setPower(SPINDLE_POWER);
            return false;
        }
    }

    public static class PrimeLaunchers implements Action {
        DcMotorEx launchLeft;
        DcMotorEx launchRight;
        double velocity;

        public PrimeLaunchers(DcMotorEx motorLeft, DcMotorEx motorRight, double velocity) {
            this.launchLeft = motorLeft;
            this.launchRight = motorRight;
            this.velocity = velocity;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launchLeft.setVelocity(velocity);
            launchRight.setVelocity(-velocity);
            return false;
        }
    }
}