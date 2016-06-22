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

namespace AtmosphereAutopilot.UI {
  [RequireComponent (typeof (CanvasGroupFader))]
  public abstract class GUIModule : MonoBehaviour {

    private CanvasGroupFader fader = null;

    public void Awake () {
      fader = GetComponent<CanvasGroupFader> ();
      fader.collapseOnFade = true;
    }

    internal void hide () {
      fader.setTransparent ();
    }

    internal void fadeIn () {
      fader.fadeIn ();
    }

    internal void fadeOut () {
      fader.fadeOut ();
    }

    internal abstract void updateGUI ();
  }
}
