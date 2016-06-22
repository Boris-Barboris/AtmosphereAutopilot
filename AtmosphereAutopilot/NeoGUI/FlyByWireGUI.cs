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
  class FlyByWireGUI : GUIModule {
    [SerializeField]
    private Toggle m_RocketModeToggle = null;
    [SerializeField]
    private Toggle m_ModerationToggle = null;
    [SerializeField]
    private Slider m_ModerationSlider = null;

    private IGUIController controller;

    internal void setController (IGUIController controller) {
      this.controller = controller;
    }

    public void setRocketMode (bool value) {
      controller.rocketMode = value;
    }

    public void setModeration (bool value) {
      controller.moderation = value;
    }

    public void setModerationValue (float value) {
      controller.moderationMult = Mathf.Pow (2f, value);
    }

    internal override void updateGUI () {
      m_RocketModeToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_ModerationToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_ModerationSlider.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.Off);
      m_RocketModeToggle.isOn = controller.rocketMode;
      m_ModerationToggle.isOn = controller.moderation;
      m_ModerationSlider.value = Mathf.Log (controller.moderationMult, 2f);
      m_RocketModeToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      m_ModerationToggle.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
      m_ModerationSlider.onValueChanged.SetPersistentListenerState (0, UnityEngine.Events.UnityEventCallState.RuntimeOnly);
    }
  }
}
