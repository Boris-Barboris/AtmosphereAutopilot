AtmosphereAutopilot
===================

Plugin for Kerbal Space Program.

Original author: Boris-Barboris.

Contributors:
* radistmorse (aka Morse on KSP forums) - Neo-GUI design and implementation.
* CraigCottingham - Cruise flight and speed control GUI refactoring, coordinate input to waypoint mode.

License: GNU GPL version 3

# General description
Atmosphere autopilot is a modular atmospheric flight control system library. It's meant to be a foundation for multiple high-level programs - "Autopilots", wich will aid KSP player in one way or another, implying atmospheric flight. Autopilots are mutually exclusive - only one or none at all may be active at the active vessel at one given moment. They are provided by the library with means of automatic reflection-based serialization\deserialization and ugly, but lazy and customizable GUI interaction.

Autopilots are modular entities. They can use basic, provided by main library components (like controllers and models), or they can define their own components and share them with other Autopilots. Those components will be called "Autopilot modules", or simply - "Modules". Every sealed child of AutopilotModule wich is not a StateController is treated by the library like a Module. Sealed StateController children are treated as Autopilots.

Stock and FAR aerodynamics are supported. Plugin is dependent on ModuleManager by sarbian, and is shipped with Mini-AVC plugin by cybutek.

# GUI concept
AA icon is placed in Application Launcher toolbar during flight. It's contents will visualize a list of all Autopilots and Modules, created for active vessel. For every vessel "Autopilot Module Manager" will be created regardless. Turning on a "MASTER SWITCH" on it's window will create required context of Autopilots and Modules for this vessel. Under the master switch all Autopilots will be listed, for the user to choose one of them as an active one. Hotkey for Master switch is letter P, autoPilot. Can be changed in Global_settings.cfg file, Autopilot_module_manager section.

Craft settings window contains shotrcuts to most used moderation and tuning parameters of the craft, as well as provides basic preset functionality. Presets are saved in "Global_settings.cfg"/settings_wnd/profiles section.

Each Autopilot and Module has it's own GUI window. All of them (even inactive ones) are accessible from AA button in Application Launcher, some of them are accessible from Autopilot window hierarchy (that's up to Autopilot developer to decide, what particular Modules should be accessible from it's GUI). Window positions are serialized (preserved between flights and game sessions) in "Global_settings.cfg" file.

# Neo-GUI
Alternative, more condensed but less powerfull way of representing AppLauncher window can be turned on by setting AtmosphereAutopilot/use_neo_gui to _true_ in Global_settings.txt config file. It is read every scene change, so the shift can be made without shutting KSP down. While it's active, "Autopilot Module Manager" is still accessible using hotkeys. Standard GUI has logical priority over Neo-GUI.

# Hotkeys
"Hotkey manager" window is placed into Application Launcher window list. It's contents are registered hotkeys, wich can be changed during runtime.
There are two main hotkeys: 
* "Master switch" - toggles Master Switch.
* Shift + "Master switch" - toggles GUI of "Autopilot Module Manager".

Others are very module-specific and will not be described here.

# Craft implications and limitations
"Control from here" part is facing prograde, with close-to-zero angle of attack bias. Planar symmetry is implied (left and right side of the plane are mirrored), as well as good degree of pitch-yaw and pitch-roll control isolation. Axial engine symmetry is strongly recommended. No wind mods are supported, as well as any mods, wich are changing control surface, rcs and engine gimbaling behaviour.

**WARNING: DO NOT USE AEROBRAKES AS CONTROL SURFACES, USE THEM ONLY AS BRAKES!**

# Default Autopilots descriptions

## Standard Fly-By-Wire
In general, FBW (Fly-By-Wire) is an abstraction Autopilot. It is designed to aid in player-controlled flight on generic (space)plane, providing a soft layer between user joystick\keyboard input and control surface outputs.
Main goals:
* Auto-trimming.
* AoA and G-force moderation.
* Sideslip handling.
* Fighting oscillations.

FBW uses three controllers - pitch, roll and yaw. Pitch is handled by "Pitch ang vel controller", roll by "Roll ang vel controller" and yaw is handled by "Sideslip controller" in plane mode, or directly by "Yaw ang vel controller" in "Rocket mode". In Rocket mode pitch and yaw axes are treated the same - it's usefull in case player wants to use FBW for rocket launches. FBW is effective only on small (<25 degrees) AoA values, though control is possible on all regimes. It's just that it's quality will degrade from inadequacy of linearization assumptions. "Moderation" button is toggling all pitch and yaw moderations - usefull for low speed VTOL action or for fighting overmoderation bugs. Pitch moderation is turned off for 2 seconds after taking-off to prevent overmoderation-related crashes.

"Coordinated turn" - pseudo-pitch hold to assist in performing coordinated turns.

Hotkeys: 
* "FBW moderation" - default hotkey for Moderation is letter O, mOderation.
* "FBW rocket mode" - default hotkey unassigned.
* "FBW coord turn" - default hotkey unassigned.

Speed control - throttle automation to maintain speed setpoint. Handeled by "Prograde thrust controller".

## Mouse Director
Mouse Director (MD) is declarative autopilot, crafted with idea to let the user to define desired airspeed direction with camera position. Autopilot then tries to comply with this surface-relative velocity setpoint. MD is inherently-linear, so only relatively small angles of attack are allowed. All AoA moderations are forcefully turned on during it's work.

MD uses "Director controller", wich uses two AoA controllers: pitch "AoA controller" and yaw "Sideslip controller", and "Roll ang vel controller" for roll. Currently, planar asymmetry of a plane is not taken into account (sideslip noise is still too noticeable in zero-lift convergence problem), sideslip is always at zero setpoint. If your craft requires nonzero sideslip to fly straight, MD is not a very good solution right now, use FbW in the _rocket mode_.

Short GUI description:

Speed control - throttle automation to maintain speed setpoint. Handeled by "Prograde thrust controller".

## Cruise Flight controller
Cruise Flight (CF) is high-level autopilot, designet for travel automation. Just like MD, CF is inherently-linear, so only relatively small angles of attack are allowed. All AoA moderations are forcefully turned on during it's work.

CF uses "Director controller" for controlling velocity vector and "Prograde thrust controller" for throttle automation.
Functions:
* Simple leveling.
* Baromethric height and airspeed control.
* Primitive waypoint functionality, picking point on planet surface (mouse click) on the map and flying to it.

Short GUI description:
* _Level_ - simple leveling regime. Upon activation, CF will save surface-relative inclination of velocity and will follow it. If altitude is not set, will keep vertical speed at zero.
* _Course_ - follows azimuth setpoint, set in field _desired course_. If altitude is not set, will keep vertical speed at zero. On high latitudes (>80 degrees) will switch to _Level_ mode.
* _Waypoint_ - primitive waypoint following. Designed for pick-and-fly functionality. When activated, _pick waypoint_ button appears under mode tabs, as well as waypoint latitude-longtitude representation and distance to it in straight line (through planet core). Waypoint control is turned off when destination is closer than 200 meters to be followed by _Level_ mode activation.
* _desired course_ - azimuth in degrees to follow in _Course_ mode.
* _Speed control_ - throttle automation to maintain speed setpoint. Handeled by "Prograde thrust controller
* _Vertical motion control_ - activate altitude or vertical speed control. Otherwise vertical speed is kept at zero.
* _Altitude_ - hold altitude, meters above sea level.
* _Vertical speed_ - hold vertical speed, meters per second.

"Advanced options" description:
* _pseudo-FLC_ - toggle for pseudo-FLC (Flight Level Change) control law for ascend. Will force CF to respect speed setpoint and craft thrust parameters when choosing ascent angle.
* _flc margin_ - default value 15 m/s. Span of pseudo-FLC algorithm relaxation region. Decrease if don't want to tolerate errors in speed. Algorithm will not converge below some minimal value, so be careful.
* _strength mult_ - default value 0.75. Will be multiplied in the runtime on Director controller's strength to restrain maneuvers. Tune to achieve slover or faster behaviour.
* _height relax time_ - default value 6.0 seconds. Time frame of proportional control law jurisdiction, related to relaxation behaviour. Tune to prevent overshooting, if really needed.
* _height relax Kp_ - gain for proportional law, decrease to slow down relaxation.
* _max climb angle_ - default value 30 degrees. Global limit on climb and drop maneuver velocity pitch. Will sometimes be exceeded, it's okay.
* _use keys_ - use pitch and yaw keys to control course and altitude\vertical speed setpoints. This flag is toggled by "CF keys input mode" hotkey.
* _hotkey course sens_ - tweak to manage course setpoint change speed.
* _hotkey altitude sens_ - tweak to manage altitude setpoint change speed.
* _hotkey vertspeed sens_ - tweak to manage vertical speed setpoint change speed.
* _hotkey vertspeed snap_ - tweak to manage vertical speed snap to zero margin.

Hotkeys:
* "Pitch keys" - alter vertical motion setpoint, altitude or vertical speed (whatever is active at the moment).
* "Yaw keys" - alter course setpoint.
* "CF keys input mode" - default hotkey is _Right Alt_, toggles whether Pitch and yaw is used to control setpoints.
* "CF vertical control" - toggles _Vertical motion control_.
* "CF altitude\vertical speed" - toggles between _Altitude_ and _Vertical speed_ modes.

# Default Modules descriptions

## Flight Model
It is a fundamental craft analysis module. It performs motion and dynamics evaluation, as well as analysis of craft aerodynamics. VTOL engine balancing is also handled by Flight Model (though it will probably change in the future). This Module will probably be used by every single other Autopilot and module.

Short GUI description (consult source code for more deatils and insight):
* Three sections for three craft principal axes, each contains:
  * _ang vel_ - angular velocity of a craft as a mechanical system of rigid bodies, radians / second. Positive for pitch up, yaw right, roll right.
  * _ang acc_ - angular acceleration, produced by numerical diffirentiation.
  * AoA - angle of attack, degrees. Positive for pitch up, yaw right. For roll it's the angle between wing chord and airspeed vector, projected on frontal plane.
* _has csurf_ - is true if Flight Model has found control surfaces on the craft. It is important for aerodynamics regressor to know it.
* Five "trainers", linear regressors. They are analyzing craft performance history and try (and fail horribly) to produce linear models of aerodynamic torques and forces. Their GUIs are filled with magic numbers you should never need to change.
* _balance engines_ - toggles engine-balancing algorithm for VTOLs. Has a hotkey.
* _balancer steering k_ - gain for attitude control using engines. Use zero to keep them static. Default value 1.
* _Lift acc_ - acceleration, provided by aerodynamic lift in the direction of plane spine.
* _Slide acc_ - acceleration, provided by aerodynamic lift in the direction of plane right wing.
* _sum acc_ - vector of total craft acceleration in PhysX reference frame.
* _pitch gravity acc_ - gravitational acceleration, projected on craft spine vector.
* _pitch engine acc_ - engines-induced acceleration, projected on craft spine vector.
* _pitch noninert acc_ - coriolis + centrifugal acceleration, projected on craft spine vector.
* _yaw gravity acc_ - gravitational acceleration, projected on craft right wing vector.
* _yaw engine acc_ - engines-induced acceleration, projected on craft right wing vector.
* _yaw noninert acc_ - coriolis + centrifugal acceleration, projected on craft right wing vector.
* _aoa virtual gain_ - default value 0.95. Gain of virtual rotation filter. Used to provide virtual craft rotation in case of interpart oscillations. 0.0 - pure control from part rotation. 1.0 - pure virtual.
* _MOI_ - moment of inertia of the craft.
* _CoM_ - center of mass of the craft in PhysX reference frame.
* _Vessel mass_ - self explanatory.
* _Reaction wheels_ - overall torque capability of reaction wheel systems.
* _RCS pos_ - estimated torque capability of RCS system when user input is positive.
* _RCS neg_ - estimated torque capability of RCS system when user input is negative.
* _e torque_ - engines-induced torque in craft principal reference frame.
* _e thrust_ - engines thrust in craft principal reference frame.
* two vectors on engine torque linear estimations. They are used to adress gimbaling capabilities of a craft.

Hotkeys:
* "Thrust balancing" - toggles _balance engines_ button.

## Director controller
Middle-level model-reference controller, follows a setpoint of surface velocity and acceleration vectors. Input: velocity vector and acceleration vector. Output: AoA, sideslip and roll angular velocity.

Short GUI description:
* _strength_ - default value 0.95. Measure of agressiveness of acceleration output of MD. Precise control multiplies output acceleration by the factor of 0.4.
* _roll stop k_ - default value 1.0, used to prevent overshooting, magic number.
* _angular error_ - error in radians between desired velocity vector and current one.
* _max angular v_ - estimate on current maneuver maximum angular velocity.
* _stop time roll_ - estimate on 90-degrees bank maneuver stop time.
* _relaxation margin_ - default value 0.01 radians. Margin of relaxed acceleration output. Magic number. Increase to fight overshooting (rarely needed).
* _angle relaxation k_ - default value 0.1. Relaxation gain, magic number. Decrease to fight oscillations.
* _max neg g_ - default value 8.0. Maximum negative g-force tolerate. May be useful for players, who are using G-force effects mods. Serialized per vessel design.
* _min rollover alt_ - default value 150.0 meters. Under this terrain altitude setpoint rolling over to prevent large negative g-force will be forbiden to decrease probability of deadly maneovers.
* _desired pitch lift_ - desired lift-induced acceleration, projected on spinal vector.
* _desired pitch acc_ - desired total acceleration, projected on spinal vector.
* _desired pitch v_ - desired angular velocity for pitch, calculated from previous value.
* _allow spine down_ - global flag to allow turning spine down to prevent negative G-force.
* _roll acc factor_ - angular acceleration factor estimate of roll rotation model.
* _roll acc filter_ - default value 4.0. filter gain for smoothing _roll acc factor_ evolution noise.
* _roll cubic K_ - default value 0.3. Cubic descent gain for roll. Increase for faster roll control, decrease for lower overshooting and oscillations.
* _roll cubic relax frame_ - default value 10.0. Relaxation frame for cubic descent phase. Magic nubmer.
* _roll relax Kp_ - default value 0.1. Relaxation gain for roll.
* _roll error filter margin_ - default value 3.0. Margin for smoothing _roll angle_ oscillations. Magic number.
* _roll error filter k_ - default value 0.5. Filter gain for _roll angle_ smoothing on relaxation regime.
* _max roll v_ - estimate of constrained maximum roll angular velocity.
* _roll error_ - current bank error in radians.
* _roll_cubic_ - true when in cubic descent regime for roll.
* _snapping boundary_ - default vaulue 3 degrees. On low bank error modes we will transition from cubic relaxation to proportional relaxation (like in roll controller wing leveler code).
* _desired aoa_ - output to "AoA controller".
* _desired sideslip_ - output to "Sideslip controller".

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
* _quadr Kp_ - default value - 0.3. Contoller uses parabolic descent model of angular velocity to it's desired value. Those descent parameters are governed by this koefficient. Larger values may cause overshoot from wrong control surface lag handling. Lower values will slow down control. Magic number.
* _kacc quadr_ - parabollic steepness of control, governed by control utilities authority and craft characteristics. Should be positive.
* _kacc smoothing_ - default value - 10.0. Filter gain for slow and smooth "kacc quadr" evolution. Magic number.
* _relaxation k_ - default value - 1.0. Controller uses relaxed linear descent on very small velocity error regimes. This koefficient governs relaxation frame size.
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
* _cubic barrier_ - default value 1.0 seconds. AoA controller uses quadratic descent profile for long evolutions and cubic for short (less than "cubic barrier" seconds). Used to prevent overshooting.
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
* _acc filter k_ - default value 1. Filter gain for acceleration moving average. Magic number.
* _relaxation acc error_ - default value 0.1 m/s^2. Error margin for filter activation. Magic number.
* _use PID_ - toggle if you want to manually tune controller, or using strange engines.
* _hotkey_speed_factor_ - tweak to change throttle hotkey sensitivity.
* _use_throttle_hotkeys_ - toggle speed setpoint handling by hotkeys.

Hotkeys:
* "Throttle keys" - alter velocity setpoint by using stock throttle hotkeys (Shift and LCntrl by default).
* "Speed control toggle" - toggles speed control ON and OFF.