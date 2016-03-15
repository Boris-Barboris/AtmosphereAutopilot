/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    class OnlineLinTrainerWindow: GUIWindow
    {
        OnlineLinTrainer trainer;

        public OnlineLinTrainerWindow(OnlineLinTrainer trainer, string name, int wnd_id, Rect window) :
            base(name, wnd_id, window)
        {
            this.trainer = trainer;
        }

        protected override void _drawGUI(int id)
        {
            GUILayout.BeginVertical();
            AutoGUI.AutoDrawObject(trainer);
            for (int i = 0; i < trainer.tasks.Count; i++)
            {
                GUILayout.Space(8.0f);
                AutoGUI.AutoDrawObject(trainer.tasks[i]);
                LinApprox linmodel = trainer.tasks[i].linmodel;
                GUILayout.Label("linmodel:", GUIStyles.labelStyleLeft);
                GUILayout.Label(String.Join(",", linmodel.pars.Select(v => v.ToString("G5")).ToArray()), GUIStyles.labelStyleCenter);
            }
            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }
}
