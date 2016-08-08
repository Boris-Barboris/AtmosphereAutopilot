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

namespace AtmosphereAutopilot.UI {
  public enum Autopilots {
    DISABLED,
    FLYBYWIRE,
    MOUSEDIR,
    CRUISECTRL
  }

  public interface IGUIController {

    // app launcher button
    bool appButtonOn { get; }

    // common
    Autopilots currentAutopilot { get; set; }

    bool speedControl { get; set; }
    float speed { get; set; }

    // fly-by-wire
    bool rocketMode { get; set; }
    bool moderation { get; set; }
    float moderationMult { get; set; }

    // cruise control
    bool altitudeControl { get; set; }
    float altitude { get; set; }
    float vesselAltitude { get; set; }
    bool pickingWaypoint { get; }
    bool waypointIsSet { get; }
    float distToWaypoint { get; }

    void pickWaypoint ();
    void cancelWaypointPick ();
    void removeWaypoint ();
  }
}