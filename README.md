AtmosphereAutopilot
===================

Plugin for Kerbal Space Program.

Original author: Boris-Barboris.

Contributors:

License: GNU GPL version 3

# General description
Atmosphere autopilot is a modular atmospheric flight control system library. It's meant to be a foundation for multiple high-level programs - "Autopilots", wich will aid KSP player in one way or another, implying atmospheric flight. Autopilots are mutually exclusive - only one or none at all may be active at the active vessel at one given moment. They are provided by the library with means of automatic reflection-based serialization\deserialization and ugly, but lazy and customizable GUI interaction.

Autopilots are modular entities. They can use basic, provided by main library components (like controllers and models), or they can define their own components and share them with other Autopilots. Those components will be called "Autopilot modules", or simply - "Modules". Every sealed child of AutopilotModule wich is not a StateController is treated by the library like a Module. Sealed StateController children are treated as Autopilots.

Stock and FAR aerodynamics are supported. Plugin is dependent on ModuleManager by sarbian.

# GUI concept
AA icon is placed in Application Launcher toolbar during flight. It's contents will visualize a list of all Autopilots and Modules, created for active vessel. For every vessel "Autopilot Module Manager" will be created regardless. Turning on a "MASTER SWITCH" on it's window will create required context of Autopilots and Modules for this vessel. Under the master switch all Autopilots will be listed, for the user to choose one of them as an active one. Each Autopilot and Module has it's own GUI window. All of them (even inactive ones) are accessible from AA button in Application Launcher, some of them are accessible from Autopilot window hierarchy (that's up to Autopilot developer to decide, what particular Modules should be accessible from it's GUI). Window positions are serialized (preserved between flights and game sessions) in "Global_settings.cfg" file.

# Craft implications and limitations
"Control from here" part is facing prograde, with close-to-zero angle of attack bias. Planar symmetry is implied (left and right side of the plane are mirrored), as well as good degree of pitch-yaw and pitch-roll control isolation. Axial engine symmetry is strongly recommended. No wind mods are supported, as well as any mods besides Stock Bug Fix Modules, wich are changing control surface, rcs and engine gimbaling behaviour.

**WARNING: DO NOT USE AEROBRAKES AS CONTROL SURFACES, USE THEM ONLY AS BRAKES!**

# Default Autopilots descriptions

## Standard Fly-By-Wire
In general, FBW (Fly-By-Wire) is an abstraction Autopilot. It is designed to aid in player-controlled flight on generic (space)plane, providing a soft layer between user joystick\keyboard input and control surface outputs.
Main goals:
* Auto-trimming.
* AoA and G-force moderation.
* Sideslip handling.
* Fighting oscillations.

FBW uses three controllers - pitch, roll and yaw. Pitch is handled by "Pitch ang vel controller", roll by "Roll ang vel controller" and yaw is handled by "Sideslip controller" in plane mode, or directly by "Yaw ang vel controller" in "Rocket mode". In Rocket mode pitch and yaw axes are treated the same - it's usefull in case player wants to use FBW for rocket launches. FBW is effective only on small (<25 degrees) AoA values, though control is possible on all regimes. It's just that it's quality will degrade from inadequacy of linearization assumptions. "Moderation" button is toggling all pitch and yaw moderations - usefull for low speed VTOL action or for fighting overmoderation bugs. 

Hotkey for FBW is letter P, autoPilot. Hardcoded.
Default hotkey for Moderation is letter O, mOderation. Can be changed in Global_settings.cfg file.

Cruise control - throttle automation to maintain speed setpoint. Handeled by "Prograde thrust controller".

# Default Modules descriptions

## Flight Model
It is a fundamental craft analysis module. It performs motion and dynamics evaluation, as well as analysis of craft aerodynamics. This Module will probably be used by every single other Autopilot and module.

Short GUI description (consult source code for more deatils and insight):
* Three sections for three craft principal axes, each contains:
  * _ang vel_ - angular velocity of a craft as a mechanical system of rigid bodies, radians / second. Positive for pitch up, yaw right, roll right.
  * _ang acc_ - angular acceleration, produced by numerical diffirentiation.
  * AoA - angle of attack, degrees. Positive for pitch up, yaw right. For roll it's the angle between wing chord and airspeed vector, projected on frontal plane.
* _has csurf_ - is true if Flight Model has found control surfaces on the craft. It is important for aerodynamics regressor to know it.
* Five "trainers", linear regressors. They are analyzing craft performance history and try (and fail horribly) to produce linear models of aerodynamic torques and forces. Their GUIs are filled with magic numbers you should never need to change.
* _Lift acc_ - acceleration, provided by aerodynamic lift in the direction of plane spine.
* _Slide acc_ - acceleration, provided by aerodynamic lift in the direction of plane right wing.
* _sum acc_ - vector of total craft acceleration in PhysX reference frame.
* _pitch gravity acc_ - gravitational acceleration, projected on craft spine vector.
* _pitch engine acc_ - engines-induced acceleration, projected on craft spine vector.
* _pitch noninert acc_ - coriolis + centrifugal acceleration, projected on craft spine vector.
* _yaw gravity acc_ - gravitational acceleration, projected on craft right wing vector.
* _yaw engine acc_ - engines-induced acceleration, projected on craft right wing vector.
* _yaw noninert acc_ - coriolis + centrifugal acceleration, projected on craft right wing vector.
* _MOI_ - moment of inertia of the craft.
* _CoM_ - center of mass of the craft in PhysX reference frame.
* _Vessel mass_ - self explanatory.
* _Reaction wheels_ - overall torque capability of reaction wheel systems.
* _RCS pos_ - estimated torque capability of RCS system when user input is positive.
* _RCS neg_ - estimated torque capability of RCS system when user input is negative.
* _e torque_ - engines-induced torque in craft principal reference frame.
* _e thrust_ - engines thrust in craft principal reference frame.
* two vectors on engine torque linear estimations. They are used to adress gimbaling capabilities of a craft.

## Pitch, roll and yaw angular acceleration controllers
Low level model-reference angular acceleration controllers. Input: desired angular acceleration. Output: pitch\roll\yaw control state.

Short GUI description:
* _Csurf output_ - current expected virtual control surface position, wich is usually lagged from control signal.
* _write telemetry_ button - primitive logging capability for further matlab analysis. .csv logs are saved in KSP\Resources directory to be read by plotter.m viewer. It is a debug utility.
* _desired acc_ - desired acceleration, passed to this controller from above.
* _model predicted acc_ - predicted by model acceleration for the next frame.
* _authority_ - linear axial authority, complicated thing, do not bother. Should always be positive though.
* _angular acc_ - angular acceleration, duplicate of Flight Model _ang acc_ field.
* _output_ - control state output, is passed to vessel in FlightCtrlState object.

## Pitch and yaw angular velocity controllers
Model-reference controllers, that perform pitch and yaw angular velocity control with respect to moderation and controllability restrictions. Input: [-1, 1] user input or desired angular velocity. Output: desired angular acceleration, passed to angular acceleration controller.

When navball is in surface mode, controller is dealing with surface-oriented reference frame. Zero input will keep zero velocity relative to ground - useful on planes. In orbit mode inertial reference frame will be used - usefull for spacecrafts. Precision mode (CAPS LOCK) divides input by the factor of 3 to provide more precise control, or to aid with control on high physical warp regimes.

Short GUI description:
* _Auto trim_ button - turn on of you want control trim to preserve after controller shutdown. Off by default.
* _max\min input aoa_ - estimated maximum angle of attack (radians), achievable by locking control to 1.0 or -1.0. When craft is statically unstable, this value is 0.6 of the controllability region boundary - it helps to stay reliable on unstable planes.
* _max\min input v_ - equilibrium angular velocities on max\min input aoa flight regimes.
* _max\min g aoa_ - estimated maximum angle of attack considering g-force moderation.
* _max\min g v_ - respective equilibrium angular velocities.
* _max\min aoa v_ - equlibrium angular velocities for set by user AoA limit.
* _moder filter_ - default value - 3.0. Used to filter out rapid changes or oscillations in flight model to provide more smooth boundary condition evolution. Magic number.
* _quadr Kp_ - default value - 0.3. Contoller uses parabolic descend model of angular velocity to it's desired value. Those descend parameters are governed by this koefficient. Larger values may cause overshoot from wrong control surface lag handling. Lower values will slow down control. Magic number.
* _kacc quadr_ - parabollic steepness of control, governed by control utilities authority and craft characteristics. Should be positive.
* _kacc smoothing_ - default value - 10.0. Filter gain for slow and smooth "kacc quadr" evolution. Magic number.
* _relaxation k_ - default value - 1.0. Controller uses relaxed linear descend on very small velocity error regimes. This koefficient governs relaxation frame size.
* _relaxation Kp_ - default value - 0.5. Relaxation gain itself.
* _relaxation frame_ - default value - 1. How many velocity frames will be averaged as current angular velocity. This is an old deprecated way of fighting oscillations, keep it 1.
* _relax count_ - for how many frames velocity is in relaxation state.
* _transit max v_ - very rough, but safe estimation of maximum non-overshooting velocity in transit maneuver (from 0.0 AoA to maximum AoA).
* _res max\min aoa_ - AoA limits, that will actually govern controller in the current frame. Are chosed as the most strict ones from previously listed.
* _res max\min v_ - respective equilibrium angular velocities.
* _scaled aoa_ - how far away current AoA is from it's limit.
* _scaled restr v_ - result of moderation algorithm.
* _Moderate AoA_ button - toggle angle of attack moderation. Is necessary for safe flight, but can be turned off, for example, during re-entry to provide maximum-drag profile. Required to be ON, if this controller is governed by upper-level AoA controller.
* _Moderate G-force_ button - toggle G-force moderation. Moderates centifugal acceleration of trajectory, not the full one, so G-meeter on navball will sometimes exceed maximum value (it is a correct behaviour).
* _max AoA_ - default value - 15.0 degrees. User-entered AoA limit. Recommended values [5.0, 25.0]. Serialized on per-design basis.
* _max G-force_ - default value - 10.0 g's. Self-explanatory. Serialized on per-design basis.
* _angular vel_ - current angular velocity of a craft in corresponding axis.
* _output acceleration_ - output, produced by controller.
* _input deriv limit_ - default value - 5.0. Artificial inertia gain. User input derivative is clamped by this value. Decrease for more inertia, increase for sharpness. Serialized globally.
* _prev input_ - previous controller input.
* _Max v construction_ - default value - 0.5 (rad/sec). Global angular velocity restriction. Is introduced to provide comfortable control by limiting vessel rotation capabilities. 0.5 is good for most crafts. Serialized on per-design basis.
* _desired v_ - desired angular velocity, not yet processed by moderation.

## Roll angular velocity controllers
Model-reference controller, that perform roll angular velocity control and wing leveling. Input: [-1, 1] user input or desired angular velocity. Output: desired angular acceleration, passed to angular acceleration controller.

Precision mode (CAPS LOCK) divides input by the factor of 3 to provide more precise control, or to aid with control on high physical warp regimes.

Short GUI description (except identical from previous module):
* _Wing leveler_ - toggle to level wings automaticly, if craft is close to zero bank angle. Zero angle is not horizontal one, but the normal one to the trajectory plane - good for leveling on non-zero pitch while yawing.
* _Snap angle_ - default value - 3.0 degrees. Decides, when to activate wing leveler.
* _angle btw hor_ - when wings are close to snapped state, this is the angle in radians. Is needed if snap angle is large and sin(x)<>x.
* _angle btw hor sin_ - sinus of the horizont angle.
* _snapping Kp_ - snapping speed gain. Default avlue - 0.25. Larger values seem to be too agressive, too large oscillate.

## AoA and Sideslip controllers
Model-reference controllers with self-explanatory names. Input: [-1, 1] user input or desired AoA. Output: desired angular velocity. Both require respective angular velocity controllers to have AoA moderation on, because it uses respective angular velocity controller limitation values as governers.

Short GUI description:
* _AoA_ - respective angle of attack in radians.
* _desired aoa_ - processed by controller input in radians.
* _output v_ - controller output.
* _desired aoa equilibr v_ - equilibrium angular velocity on desired angle of attack. For example, nosedive angular velocity on nose-heavy plane, wich will keep AoA at zero.
* _filter k_ - filter gain to smooth changes in equilibrium v estimation. Default value - 4.0.
* _relaxation frame_ - relaxation frame count, used for close-to desired behaviour. Default value - 2.
* _relaxation factor_ - default value 0.1. Proportional gain of relaxation smoothing.
* _cubic barrier_ - default value 1.0 seconds. AoA controller uses quadratic descend profile for long evolutions and cubic for short (less than "cubic barrier" seconds). Used to prevent overshooting.
* _cubic KP_ - default value 0.3. Gain for cubic profile steepness evaluation.
* _cubic mode_ - true if controller is now in cubic mode.

## Prograde thrust controller
Hybrid model-reference or PID controller. Input: desired surface velocity. Output: throttle. Can be switched to PID control and manually tuned, if user is not satisfied with it's performance.

Short GUI description:
* _pid Kp_ - if used in PID mode, it's the proportional PID gain.
* _pid Ki_ - integrad PID gain.
* _pid Kd_ - derivative PID gain.
* _desired v_ - self explanatory.
* _current v_ - self explanatory.
* _break spd margin_ - when surface speed is exceeding desired by this margin, brakes will be used. On groud breaks are used without margin.
* _Use breaks_ - controller is using "Breaks" action group.
* _prograde thrust_ - thrust vector projection on prograde direction.
* _Kp v_ - proportional gain on velocity error. Default value - 0.5, e.g. on 1 m/s error it will be 0.5 m/s^2 desired acceleration. Decrease if don't like overshooting on very slow jets.
* _acc filter k_ - default value 10. Filter gain for acceleration moving average. Magic number.
* _relaxation acc error_ - default value 0.1 m/s^2. Error margin for filter activation. Magic number.
* _use PID_ - toggle if you want to manually tune controller, or using strange engines.