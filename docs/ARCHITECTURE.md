# Architecture

## Sensor Policy

- DVL is allowed for distance estimation and body-frame horizontal velocity control.
- PingAltimeter / RangeFinder is allowed for seabed-relative altitude control in Step 3 and Step 3B.
- Pose is ground truth only. It may be logged and used for offline metrics, never for control.
- Velocity is ground truth only. It may be logged and used for offline metrics, never for control.
- IMU and Depth are available for future work but are not required by the current controllers.

## DVL Use

Step 1 integrates DVL forward velocity for distance. Step 2B and Step 2C use
DVL body-frame velocity feedback for horizontal velocity tracking. The
controllers do not receive the configured HoloOcean current vector.

## Body-Frame Command Abstraction

Step 2C introduced `BodyVelocitySetpoint`, `BodyVelocityMeasurement`, and
`BodyCommand` in `src/control/body_commands.py`. Controllers return abstract
normalized body-frame commands (`surge`, `sway`, `heave`, `yaw`) instead of
thruster vectors.

This keeps controller code independent from any simulator or hardware motor
ordering.

## HoloOcean Mixer and Real BlueROV2 Backend

`src/actuation/holoocean_bluerov2_mixer.py` converts abstract body-frame
commands into HoloOcean `control_scheme=0` 8-thruster vectors. That mapping is
simulation-specific and preserves the legacy Step 2B HoloOcean convention.

Real BlueROV2 / BlueROV2 Heavy hardware should not hard-code this motor order.
A future ArduSub/MAVLink backend should send body-axis commands to ArduSub and
let ArduSub's configured frame mixer handle motor allocation.

## Step 3 Altitude Hold

Step 3 combines DVL horizontal tracking with PingAltimeter altitude hold. The
vertical controller uses:

```text
altitude_error = desired_altitude - measured_ping_altitude
vertical_command = kp_altitude * altitude_error
```

Pose.z is logged for evaluation only.

## Step 3B Terrain Reconstruction

Step 3B runs OpenWater transects over documented seabed depressions. It still
uses only PingAltimeter for vertical control. Pose is used offline to estimate
the seabed profile:

```text
estimated_seabed_z_from_pose_ping = pose_z - ping_altitude
```

This field is for analysis and plotting only.
