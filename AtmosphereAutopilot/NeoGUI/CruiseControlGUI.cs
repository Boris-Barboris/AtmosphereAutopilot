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

using UnityEngine;
using UnityEngine.UI;

namespace AtmosphereAutopilot.UI {
  public class CruiseControlGUI : GUIModule {
    [SerializeField]
    private Text m_StatusLine = null;
    [SerializeField]
    private Toggle m_SetWaypointToggle = null;
    [SerializeField]
    internal Toggle m_AltitudeControlToggle = null;
    [SerializeField]
    internal Slider m_AltitudeControlSlider = null;
    [SerializeField]
    private Text m_AltitudeControlValue = null;

    private IGUIController controller;

    private bool setWaypointAck = false;

    internal void setController (IGUIController controller) {
      this.controller = controller;
    }

    internal override void updateGUI () {
      m_AltitudeControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_AltitudeControlSlider.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_SetWaypointToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_AltitudeControlToggle.isOn = controller.altitudeControl;
      m_AltitudeControlSlider.value = Mathf.Round(controller.altitude/500f);
      m_SetWaypointToggle.isOn = controller.pickingWaypoint;
      m_AltitudeControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      m_AltitudeControlSlider.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      m_SetWaypointToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      if (controller.altitudeControl) {
        m_AltitudeControlValue.text = (controller.altitude / 1000f).ToString ("0.0") + " km";
        m_AltitudeControlValue.alignment = TextAnchor.MiddleRight;
      } else {
        m_AltitudeControlValue.text = "OFF";
        m_AltitudeControlValue.alignment = TextAnchor.MiddleCenter;
      }
    }

    private float cacheDist = 0f;
    private bool keepDirectionLabel = false;
    private bool pickingWaypointLabel = false;

    public void Update () {
      if (controller == null)
        return;



      if (controller.pickingWaypoint) {
        if (!pickingWaypointLabel) {
          m_StatusLine.text = "picking waypoint";
          pickingWaypointLabel = true;
          keepDirectionLabel = false;
        }
      } else if (controller.waypointIsSet) {
        if (setWaypointAck) {
          m_AltitudeControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
          m_AltitudeControlToggle.isOn = false;
          m_AltitudeControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
          setWaypointAck = false;
        }
        if (pickingWaypointLabel || keepDirectionLabel || Mathf.Abs (controller.distToWaypoint - cacheDist) > 0.05) {
          cacheDist = controller.distToWaypoint;
          m_StatusLine.text = string.Format ("fly to waypoint, {0:0.0} km left", cacheDist / 1000f);
        }
        keepDirectionLabel = false;
        pickingWaypointLabel = false;
      } else {
        if (!keepDirectionLabel) {
          m_StatusLine.text = "keep direction";
          keepDirectionLabel = true;
          pickingWaypointLabel = false;
        }
      }
    }

    public void setWaypointToggle (bool value) {
      if (value)
        controller.pickWaypoint ();
      else
        controller.cancelWaypointPick ();
      setWaypointAck = value;
    }

    public void unsetWaypointButton () {
      m_SetWaypointToggle.isOn = false;
      controller.removeWaypoint ();
    }

    public void setAltitudeControl (bool value) {
      controller.altitudeControl = value;
      if (value) {
        m_AltitudeControlValue.text = (controller.altitude / 1000f).ToString ("0.0") + " km";
        m_AltitudeControlValue.alignment = TextAnchor.MiddleRight;
      } else {
        m_AltitudeControlValue.text = "OFF";
        m_AltitudeControlValue.alignment = TextAnchor.MiddleCenter;
      }
     }

    public void setAltitude (float value) {
      controller.altitude = value * 500f;
      if (controller.altitudeControl)
        m_AltitudeControlValue.text = (value / 2f).ToString ("0.0") + " km";
    }
  }
}
