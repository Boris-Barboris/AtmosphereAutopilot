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

using UnityEngine.UI;
using UnityEngine;

namespace AtmosphereAutopilot.UI {
    public class ShrinkPanel : HorizontalOrVerticalLayoutGroup {
      public override void CalculateLayoutInputHorizontal () {
        base.CalculateLayoutInputHorizontal ();
        CalcAlongAxis (0, true);
      }

      public override void CalculateLayoutInputVertical () {
        CalcAlongAxis (1, true);
        SetLayoutInputForAxis (minHeight, preferredHeight * GetComponent<RectTransform> ().localScale.y, flexibleHeight, 1);
      }

      public override void SetLayoutHorizontal () {
        SetChildrenAlongAxis (0, true);
      }

      public override void SetLayoutVertical () {
        SetChildrenAlongAxis (1, true);
      }
    }
}
