AtmosphereAutopilot
===================

Plugin for Kerbal Space Program.

Original author: Boris-Barboris.
Contributors:

# General description
Atmosphere autopilot is a modular atmospheric flight control system library. It's meant to be a foundation for multiple high-level programs - "Autopilots", wich will aid KSP player in one way or another, implying atmospheric flight. Autopilots are mutually exclusive - only one or none at all may be active at the active vessel at one given moment. They are provided by the library with means of automatic reflection-based serialization\deserialization and ugly, but lazy and customizable GUI interaction.

Autopilots are modular entities. They can use basic, provided by main library components (like controllers and models), or they can define their own components and share them with other Autopilots. Those components will be called "Autopilot modules", or simply - "Modules". Every sealed child of AutopilotModule wich is not a StateController is treated by the library like a Module. Sealed StateController children are treated as Autopilots.

Stock and FAR aerodynamics are supported. Plugin is dependent on ModuleManager by sarbian.

# GUI concept
AA icon is placed in Application Launcher toolbar during flight. It's contents will visualize a list of all Autopilots and Modules, created for active vessel. For every vessel "Autopilot Module Manager" will be created regardless. Turning on a "MASTER SWITCH" on it's window will create required context of Autopilots and Modules for this vessel. Under the master switch all Autopilots will be listed, for the user to choose one of them as an active one. Each Autopilot and Module has it's own GUI window. All of them (even inactive ones) are accessible from AA button in Application Launcher, some of them are accessible from Autopilot window hierarchy (that's up to Autopilot developer to decide, what particular Modules should be accessible from it's GUI). Window positions are serialized (preserved between flights and game sessions) in "Global_settings.cfg" file.

# Craft implications and limitations
"Control from here" part is facing prograde, with zero angle of attack. Planar symmetry is implied (left and right side of the plane are mirrored), as well as good degree of pitch-yaw and pitch-roll control isolation. Axial engine symmetry is strongly recommended. No wind mods are supported, as well as any mods besides Stock Bug Fix Modules, wich are changing control surface, rcs and engine gimbaling behaviour.

**WARNING: DO NOT USE AEROBRAKES AS CONTROL SURFACES, USE THEM ONLY AS BRAKES!**

# Default Autopilots descriptions

## Standard Fly-By-Wire
In general, FBW (Fly-By-Wire) is an abstraction Autopilot. It is designed to aid in player-controlled flight on generic (space)plane, providing a soft layer between user joystick\keyboard input and control surface outputs.
Main goals:
* Auto-trimming.
* AoA and G-force moderation.
* Sideslip handling.
* Fighting oscillations.

FBW uses three controllers - pitch, roll and yaw. Pitch is handled by "Pitch ang vel controller", roll by "Roll ang vel controller" and yaw is handled by "Sideslip controller" in plane mode, or directly by "Yaw ang vel controller" in "Rocket mode". In Rocket mode pitch and yaw axes are treated the same - it's usefull in case player wants to use FBW for rocket launches. FBW is effective only on small (<25 degrees) AoA values, though control is possible on all regimes. It's just that it's quality will degrade from inadequacy of linearization assumptions.

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
* Three "cpu" integer fields. If they're blinking in 0-10 diapasone - background thread is active and is busy with regressor code.
* _Lift acc_ - acceleration, provided by aerodynamic lift in the direction of plane spine.
* _Slide acc_ - acceleration, provided by aerodynamic lift in the direction of plane right wind.
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
* four vectors on engine torque linear estimations. They are used to adress gimbaling capabilities of a craft.

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
Model-reference controllers, that perform pitch and yaw angular velocity control with respect to moderation and controllability restrictions. They are designed for both higher-level input and direct user input.