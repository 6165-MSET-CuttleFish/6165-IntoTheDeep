package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.Drive;
import org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive;
import org.firstinspires.ftc.teamcode.roadrunner.unused.TankDrive;
import org.firstinspires.ftc.teamcode.roadrunner.unused.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadrunner.unused.TwoDeadWheelLocalizer;

import java.util.*;

public final class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = PinpointDrive.class; // TODO: change to your drive class i.e. PinpointDrive if using pinpoint

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf;
        if (DRIVE_CLASS.equals(PinpointDrive.class)) {
                dvf = hardwareMap -> {
                    PinpointDrive pd = new PinpointDrive(hardwareMap, new Pose2d(0, 0, 0));

                    List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                    List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                    parEncs.add(new PinpointEncoder(pd.pinpoint,false, pd.bl));
                    perpEncs.add(new PinpointEncoder(pd.pinpoint,true, pd.bl));

                    return new DriveView(
                            DriveType.MECANUM,
                            Drive.PARAMS.inPerTick,
                            Drive.PARAMS.maxWheelVel,
                            Drive.PARAMS.minProfileAccel,
                            Drive.PARAMS.maxProfileAccel,
                            hardwareMap.getAll(LynxModule.class),
                            Arrays.asList(
                                    pd.fl,
                                    pd.bl
                            ),
                            Arrays.asList(
                                    pd.fr,
                                    pd.br
                            ),
                            leftEncs,
                            rightEncs,
                            parEncs,
                            perpEncs,
                            pd.lazyImu,
                            pd.voltageSensor,
                            () -> new MotorFeedforward(Drive.PARAMS.kS,
                                    Drive.PARAMS.kV / Drive.PARAMS.inPerTick,
                                    Drive.PARAMS.kA / Drive.PARAMS.inPerTick)
                    );
                };
        } else if (DRIVE_CLASS.equals(Drive.class)) {
            dvf = hardwareMap -> {
                Drive md = new Drive(hardwareMap, new Pose2d(0, 0, 0));

                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (md.localizer instanceof Drive.DriveLocalizer) {
                    Drive.DriveLocalizer dl = (Drive.DriveLocalizer) md.localizer;
                    leftEncs.add(dl.leftFront);
                    leftEncs.add(dl.leftBack);
                    rightEncs.add(dl.rightFront);
                    rightEncs.add(dl.rightBack);
                } else if (md.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (md.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) md.localizer;
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("unknown localizer: " + md.localizer.getClass().getName());
                }

                return new DriveView(
                    DriveType.MECANUM,
                        Drive.PARAMS.inPerTick,
                        Drive.PARAMS.maxWheelVel,
                        Drive.PARAMS.minProfileAccel,
                        Drive.PARAMS.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        Arrays.asList(
                                md.fl,
                                md.bl
                        ),
                        Arrays.asList(
                                md.fr,
                                md.br
                        ),
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        md.lazyImu,
                        md.voltageSensor,
                        () -> new MotorFeedforward(Drive.PARAMS.kS,
                                Drive.PARAMS.kV / Drive.PARAMS.inPerTick,
                                Drive.PARAMS.kA / Drive.PARAMS.inPerTick)
                );
            };
        } else if (DRIVE_CLASS.equals(TankDrive.class)) {
            dvf = hardwareMap -> {
                TankDrive td = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

                List<Encoder> leftEncs = new ArrayList<>(), rightEncs = new ArrayList<>();
                List<Encoder> parEncs = new ArrayList<>(), perpEncs = new ArrayList<>();
                if (td.localizer instanceof TankDrive.DriveLocalizer) {
                    TankDrive.DriveLocalizer dl = (TankDrive.DriveLocalizer) td.localizer;
                    leftEncs.addAll(dl.leftEncs);
                    rightEncs.addAll(dl.rightEncs);
                } else if (td.localizer instanceof ThreeDeadWheelLocalizer) {
                    ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) td.localizer;
                    parEncs.add(dl.par0);
                    parEncs.add(dl.par1);
                    perpEncs.add(dl.perp);
                } else if (td.localizer instanceof TwoDeadWheelLocalizer) {
                    TwoDeadWheelLocalizer dl = (TwoDeadWheelLocalizer) td.localizer;
                    parEncs.add(dl.par);
                    perpEncs.add(dl.perp);
                } else {
                    throw new RuntimeException("unknown localizer: " + td.localizer.getClass().getName());
                }

                return new DriveView(
                    DriveType.TANK,
                        TankDrive.PARAMS.inPerTick,
                        TankDrive.PARAMS.maxWheelVel,
                        TankDrive.PARAMS.minProfileAccel,
                        TankDrive.PARAMS.maxProfileAccel,
                        hardwareMap.getAll(LynxModule.class),
                        td.leftMotors,
                        td.rightMotors,
                        leftEncs,
                        rightEncs,
                        parEncs,
                        perpEncs,
                        td.lazyImu,
                        td.voltageSensor,
                        () -> new MotorFeedforward(TankDrive.PARAMS.kS,
                                TankDrive.PARAMS.kV / TankDrive.PARAMS.inPerTick,
                                TankDrive.PARAMS.kA / TankDrive.PARAMS.inPerTick)
                );
            };
        } else {
            throw new RuntimeException();
        }

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(LateralFeedbackTuner.class), LateralFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class,
                    LateralFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}
