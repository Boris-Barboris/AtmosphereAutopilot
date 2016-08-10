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
using UnityEngine.EventSystems;

namespace AtmosphereAutopilot.UI {
  [RequireComponent (typeof (CanvasGroupFader))]
  public class MainMenuGUI : MonoBehaviour, IPointerEnterHandler, IPointerExitHandler, IPointerDownHandler {
    [SerializeField]
    private Toggle m_FlyByWireToggle = null;
    [SerializeField]
    private Toggle m_MouseDirectorToggle = null;
    [SerializeField]
    private Toggle m_CruiseControlToggle = null;
    [SerializeField]
    private CanvasGroupFader m_SpeedControlPanel = null;
    [SerializeField]
    private Toggle m_SpeedControlToggle = null;
    [SerializeField]
    private Slider m_SpeedControlSlider = null;
    [SerializeField]
    private Text m_SpeedControlValue = null;

    [SerializeField]
    private CruiseControlGUI m_CruiseControl = null;
    [SerializeField]
    private FlyByWireGUI m_FBWControl = null;

    private CanvasGroupFader _fader = null;
    private RectTransform _rectTransform = null;
    private ToggleGroup _toggleGroup = null;

    private CanvasGroupFader fader {
      get {
        if (_fader == null)
          _fader = GetComponent<CanvasGroupFader> ();
        return _fader;
      }
    }
    private RectTransform rectTransform {
      get {
        if (_rectTransform == null)
          _rectTransform = GetComponent<RectTransform> ();
        return _rectTransform;
      }
    }
    private ToggleGroup toggleGroup {
      get {
        if (_toggleGroup == null)
          _toggleGroup = GetComponent<ToggleGroup> ();
        return _toggleGroup;
      }
    }
    private IGUIController controller = null;
    
    public void Awake () {
      m_SpeedControlPanel.collapseOnFade = true;
      if (fader == null)
        return;
      fader.setTransparent ();
      fader.fadeIn ();
    }

    public void OnPointerEnter (PointerEventData eventData) {
      fader.fadeIn ();
    }

    public void OnPointerExit (PointerEventData eventData) {
      if (controller != null && controller.appButtonOn == false)
        fader.fadeCloseSlow ();
    }

    public void fadeIn () {
      fader.fadeIn ();
    }

    public void fadeOut () {
      if (!fader.IsFadingOut)
        fader.fadeClose ();
    }

    public void OnPointerDown (PointerEventData data) {
      rectTransform.SetAsLastSibling ();
    }

    public void setController (IGUIController controller) {
      this.controller = controller;
      m_CruiseControl.setController (controller);
      m_FBWControl.setController (controller);
      if (controller.currentAutopilot == Autopilots.DISABLED) {
        m_SpeedControlPanel.setTransparent ();
        m_CruiseControl.hide ();
        m_FBWControl.hide ();
      } else if (controller.currentAutopilot == Autopilots.FLYBYWIRE) {
        m_CruiseControl.hide ();
      } else if (controller.currentAutopilot == Autopilots.MOUSEDIR) {
        m_CruiseControl.hide ();
        m_FBWControl.hide ();
      } else if (controller.currentAutopilot == Autopilots.CRUISECTRL) {
        m_FBWControl.hide ();
      }
      m_CruiseControl.m_AltitudeControlToggle.isOn = false;
      m_CruiseControl.m_AltitudeControlSlider.value = Mathf.Clamp(Mathf.Round(controller.vesselAltitude / 500f), 0.0f, 50.0f);
      updateGUI ();
    }

    public void updateGUI () {
      m_FlyByWireToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_MouseDirectorToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_CruiseControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_FlyByWireToggle.isOn = controller.currentAutopilot == Autopilots.FLYBYWIRE;
      m_MouseDirectorToggle.isOn = controller.currentAutopilot == Autopilots.MOUSEDIR;
      m_CruiseControlToggle.isOn = controller.currentAutopilot == Autopilots.CRUISECTRL;
      m_FlyByWireToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      m_MouseDirectorToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      m_CruiseControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      if (controller.currentAutopilot == Autopilots.DISABLED) {
        m_SpeedControlPanel.fadeOut ();
        m_CruiseControl.fadeOut ();
        m_FBWControl.fadeOut ();
      } else {
        m_SpeedControlPanel.fadeIn ();
        m_SpeedControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
        m_SpeedControlSlider.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
        m_SpeedControlToggle.isOn = controller.speedControl;
        m_SpeedControlSlider.value = controller.speed;
        m_SpeedControlToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
        m_SpeedControlSlider.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
        if (controller.speedControl) {
          m_SpeedControlValue.text = controller.speed.ToString ("0.#") + " m/s";
          m_SpeedControlValue.alignment = TextAnchor.MiddleRight;
        } else {
          m_SpeedControlValue.text = "OFF";
          m_SpeedControlValue.alignment = TextAnchor.MiddleCenter;
        }
        if (controller.currentAutopilot == Autopilots.FLYBYWIRE) {
          m_CruiseControl.fadeOut ();
          m_FBWControl.fadeIn ();
          m_FBWControl.updateGUI ();
        } else if (controller.currentAutopilot == Autopilots.MOUSEDIR) {
          m_CruiseControl.fadeOut ();
          m_FBWControl.fadeOut ();
        } else if (controller.currentAutopilot == Autopilots.CRUISECTRL) {
          m_FBWControl.fadeOut ();
          m_CruiseControl.fadeIn ();
          m_CruiseControl.updateGUI ();
        }
      }
    }

        public void updateSpeed()
        {
            m_SpeedControlToggle.onValueChanged.SetPersistentListenerState(0, UnityEngine.Events.UnityEventCallState.Off);
            m_SpeedControlSlider.onValueChanged.SetPersistentListenerState(0, UnityEngine.Events.UnityEventCallState.Off);
            m_SpeedControlToggle.isOn = controller.speedControl;
            m_SpeedControlSlider.value = controller.speed;
            m_SpeedControlToggle.onValueChanged.SetPersistentListenerState(0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
            m_SpeedControlSlider.onValueChanged.SetPersistentListenerState(0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
            if (controller.speedControl)
            {
                m_SpeedControlValue.text = controller.speed.ToString("0.#") + " m/s";
                m_SpeedControlValue.alignment = TextAnchor.MiddleRight;
            }
            else
            {
                m_SpeedControlValue.text = "OFF";
                m_SpeedControlValue.alignment = TextAnchor.MiddleCenter;
            }
        }

    public void toggleFlyByWire (bool value) {
      if (value)
        controller.currentAutopilot = Autopilots.FLYBYWIRE;
      else
        if (!toggleGroup.AnyTogglesOn ())
          controller.currentAutopilot = Autopilots.DISABLED;
        else
          return;
      updateGUI ();
    }
    public void toggleCruiseControl (bool value) {
        if (value)
        {
            controller.currentAutopilot = Autopilots.CRUISECTRL;
            m_CruiseControl.m_AltitudeControlToggle.isOn = false;
            m_CruiseControl.m_AltitudeControlSlider.value = Mathf.Clamp(Mathf.Round(controller.vesselAltitude / 500f), 0.0f, 50.0f);
        }
        else
            if (!toggleGroup.AnyTogglesOn())
                controller.currentAutopilot = Autopilots.DISABLED;
        else
            return;
        updateGUI ();
    }
    public void toggleMouseDirector (bool value) {
      if (value)
        controller.currentAutopilot = Autopilots.MOUSEDIR;
      else
        if (!toggleGroup.AnyTogglesOn ())
          controller.currentAutopilot = Autopilots.DISABLED;
        else
          return;
      updateGUI ();
    }
    public void toggleSpeedControl (bool value) {
      controller.speedControl = value;
      updateGUI ();
    }
    public void setSpeed (float value) {
      controller.speed = value;
      if (controller.speedControl) {
        m_SpeedControlValue.text = controller.speed.ToString ("0.#") + " m/s";
        m_SpeedControlValue.alignment = TextAnchor.MiddleRight;
      } else {
        m_SpeedControlValue.text = "OFF";
        m_SpeedControlValue.alignment = TextAnchor.MiddleCenter;
      }
    }
  }
}
