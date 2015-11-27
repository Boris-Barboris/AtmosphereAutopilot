using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace AtmosphereAutopilot
{

    [KSPAddon(KSPAddon.Startup.Instantly, true)]
    class GimbalRearranger: MonoBehaviour
    {
        bool done = false;

        internal void Awake()
        {
            if (done)
                return;

            List<LoadingSystem> list = LoadingScreen.Instance.loaders;
            if (list != null)
            {
                GameObject go = new GameObject("GimbalRearranger");
                Rearranger ra = go.AddComponent<Rearranger>();
                list.Insert(list.Count - 1, ra);
            }

            DontDestroyOnLoad(this);
        }

        class Rearranger : LoadingSystem
        {
            bool ready = false;

            public override bool IsReady()
            {
                return ready;
            }

            public override void StartLoad()
            {
                // move all gimbal modules before engine modules to fix stock error of applying
                // thrust before turning gimbal
                var part_configs = GameDatabase.Instance.GetConfigNodes("PART");
                foreach (var part in part_configs)
                {
                    ConfigNode gimbal_node;
                    if ((gimbal_node = part.nodes.GetNode("MODULE", "name", "ModuleGimbal")) != null)
                    {
                        part.RemoveNode(gimbal_node);
                        List<ConfigNode> backup_nodes = new List<ConfigNode>();

                        for (int i = 0; i < part.nodes.Count; i++)
                            backup_nodes.Add(part.nodes[i]);
                        part.nodes.Clear();

                        part.nodes.Add(gimbal_node);
                        for (int i = 0; i < backup_nodes.Count; i++)
                            part.nodes.Add(backup_nodes[i]);
                        Debug.Log("[AtmosphereAutopilot]: part '" + part.GetValue("name") + "' config node contains ModuleGimbal. Nodes count is " +
                            (backup_nodes.Count + 1).ToString());
                    }
                }
                ready = true;
            }
        }
    }
}
