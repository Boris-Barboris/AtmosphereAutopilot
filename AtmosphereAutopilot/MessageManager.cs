using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{
    static class MessageManager
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
