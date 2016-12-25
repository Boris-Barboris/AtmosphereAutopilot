/*
 * Atmosphere Autopilot, plugin for Kerbal Space Program.
 * 
 * Copyright (C) 2016, George Sedov.
 * 
 * Atmosphere Autopilot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * Atmosphere Autopilot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
 */

namespace AtmosphereAutopilot {
  internal class NeoGUIController : UI.IGUIController {
    private AtmosphereAutopilot parent;

    private TopModuleManager masterAP = null;
    private StandardFlyByWire fbwAP = null;
    private CruiseController ccAP = null;
    private MouseDirector dcAP = null;
    private ProgradeThrustController speedAP = null;
    private PitchAngularVelocityController pvc = null;
    private YawAngularVelocityController yvc = null;

    private void findModules () {
      if (parent.ActiveVessel == null || parent.autopilot_module_lists.ContainsKey (parent.ActiveVessel) == false)
        return;

      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (TopModuleManager)))
        masterAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (TopModuleManager)] as TopModuleManager;
      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (StandardFlyByWire)))
        fbwAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (StandardFlyByWire)] as StandardFlyByWire;
      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (CruiseController)))
        ccAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (CruiseController)] as CruiseController;
      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (MouseDirector)))
        dcAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (MouseDirector)] as MouseDirector;
      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (ProgradeThrustController)))
        speedAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (ProgradeThrustController)] as ProgradeThrustController;
      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (PitchAngularVelocityController)))
        pvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (PitchAngularVelocityController)] as PitchAngularVelocityController;
      if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (YawAngularVelocityController)))
        yvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (YawAngularVelocityController)] as YawAngularVelocityController;
    }

    internal NeoGUIController (AtmosphereAutopilot parent) {
      this.parent = parent;

      findModules ();
    }

    public bool appButtonOn {
      get {
        return parent.launcherButtonState;
      }
    }

    public UI.Autopilots currentAutopilot {
      get {
        findModules ();
        if (masterAP != null && masterAP.Active == false)
          return UI.Autopilots.DISABLED;
        if (fbwAP != null && fbwAP.Active)
          return UI.Autopilots.FLYBYWIRE;
        if (ccAP != null && ccAP.Active)
          return UI.Autopilots.CRUISECTRL;
        if (dcAP != null && dcAP.Active)
          return UI.Autopilots.MOUSEDIR;
        return UI.Autopilots.DISABLED;
      }

      set {
        if (masterAP == null)
          return;

        switch (value) {
          case UI.Autopilots.DISABLED:
            fbwAP = null;
            ccAP = null;
            dcAP = null;
            speedAP = null;
            pvc = null;
            yvc = null;
            masterAP.Active = false;
            parent.setLauncherOnOffIcon (false);
            return;
          case UI.Autopilots.FLYBYWIRE:
            fbwAP = masterAP.activateAutopilot (typeof (StandardFlyByWire)) as StandardFlyByWire;
            ccAP = null;
            dcAP = null;
            break;
          case UI.Autopilots.CRUISECTRL:
            fbwAP = null;
            ccAP = masterAP.activateAutopilot (typeof (CruiseController)) as CruiseController;
            dcAP = null;
            break;
          case UI.Autopilots.MOUSEDIR:
            fbwAP = null;
            ccAP = null;
            dcAP = masterAP.activateAutopilot (typeof (MouseDirector)) as MouseDirector;
            break;
        }
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (ProgradeThrustController)))
          speedAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (ProgradeThrustController)] as ProgradeThrustController;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (PitchAngularVelocityController)))
          pvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (PitchAngularVelocityController)] as PitchAngularVelocityController;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (YawAngularVelocityController)))
          yvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (YawAngularVelocityController)] as YawAngularVelocityController;

        parent.setLauncherOnOffIcon (masterAP.Active);
      }
    }

    #region Speed Control

    public bool speedControl {
      get {
        return speedAP != null && speedAP.spd_control_enabled;
      }

      set {
        if (speedAP != null)
          speedAP.spd_control_enabled = value;
      }
    }

    public float speed {
      get {
        if (speedAP == null)
          return 0f;
        return speedAP.setpoint.mps();
      }

      set {
        if (parent.ActiveVessel != null) {
          speedAP.setpoint_field.Value = value;
          speedAP.chosen_spd_mode = 1;
          speedAP.setpoint = new SpeedSetpoint(SpeedType.MetersPerSecond, value, parent.ActiveVessel);
        }
      }
    }

    #endregion

    #region Fly-By-Wire

    public bool rocketMode {
      get {
        return fbwAP != null && fbwAP.rocket_mode;
      }
      set {
        if (fbwAP != null)
          fbwAP.rocket_mode = value;
      }
    }

    // the "normal" value for all the limits, hardcoded
    private const float moderationNorm = 10f;

    private void setModerationLimits (float limit) {
      if (pvc != null) {
        pvc.max_aoa = limit;
        pvc.max_g_force = limit;
      }
      if (yvc != null) {
        yvc.max_aoa = limit;
        yvc.max_g_force = limit;
      }
    }

    public bool moderation {
      get {
        return fbwAP != null && fbwAP.moderation_switch;
      }
      set {
        if (fbwAP != null)
          fbwAP.moderation_switch = value;
      }
    }

    public float moderationMult {
      get {
        if (pvc == null)
          return 1f;
        //take pitch aoa limit as the master
        float value = pvc.max_aoa;
        setModerationLimits (value);
        return value / moderationNorm;
      }
      set {
        setModerationLimits (value * moderationNorm);
      }
    }

    #endregion

    #region Cruise Control

    public bool altitudeControl {
      get {
        return ccAP != null && ccAP.vertical_control;
      }
      set {
        if (ccAP != null)
          ccAP.vertical_control = value;
      }
    }

    public float altitude {
      get {
        if (ccAP == null)
          return 0f;
        return ccAP.desired_altitude;
      }
      set {
        if (ccAP != null)
          ccAP.desired_altitude.Value = value;
      }
    }

    public float vesselAltitude
    {
        get
        {
            if (ccAP == null)
                return 0f;
            return (float)FlightGlobals.ActiveVessel.altitude;
        }
        set { }
    }

        public float distToWaypoint {
      get {
        if (ccAP == null || ccAP.current_mode != CruiseController.CruiseMode.Waypoint)
          return -1f;

        return (float)ccAP.dist_to_dest;
      }
    }

    public bool pickingWaypoint {
      get {
        return ccAP != null && ccAP.current_mode == CruiseController.CruiseMode.Waypoint && ccAP.picking_waypoint;
      }
    }

    public bool waypointIsSet {
      get {
        return ccAP != null && ccAP.current_mode == CruiseController.CruiseMode.Waypoint && ccAP.waypoint_entered;
      }
    }

    public void pickWaypoint () {
      if (ccAP == null)
        return;
      ccAP.WaypointMode = true;
      MapView.EnterMapView ();
      ccAP.picking_waypoint = true;
    }

    public void cancelWaypointPick () {
      if (ccAP == null)
        return;
      ccAP.picking_waypoint = false;
    }

    public void removeWaypoint () {
      if (ccAP == null)
        return;
      ccAP.picking_waypoint = false;
      ccAP.LevelFlightMode = true;
    }

    #endregion

  }
}
