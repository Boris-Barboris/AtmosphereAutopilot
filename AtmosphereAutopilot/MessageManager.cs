/*
Copyright 2015, Boris-Barboris

This file is part of Atmosphere Autopilot.
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
    public static class MessageManager
    {
        static Dictionary<string, ScreenMessage> screenMessages = new Dictionary<string, ScreenMessage>();

        public static void post_status_message(string message)
        {
            ScreenMessage msg = screenMessages.ContainsKey(message) ? 
                screenMessages[message] : new ScreenMessage(message, 3.0f, ScreenMessageStyle.UPPER_RIGHT);
            screenMessages[message] = msg;
            ScreenMessages.PostScreenMessage(msg);
        }

        public static void post_quick_message(string message)
        {
            ScreenMessages.PostScreenMessage(message, 3.0f, ScreenMessageStyle.UPPER_CENTER);
        }
    }
}
