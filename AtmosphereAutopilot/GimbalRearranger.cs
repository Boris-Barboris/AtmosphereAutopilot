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
                        move_node_first(gimbal_node, part);
                        Debug.Log("[AtmosphereAutopilot]: part '" + part.GetValue("name") + "' config node contains ModuleGimbal, moving it");
                    }
                    else
                        if ((gimbal_node = part.nodes.GetNode("MODULE", "name", "KM_Gimbal_3")) != null)
                        {
                            move_node_first(gimbal_node, part);
                            Debug.Log("[AtmosphereAutopilot]: part '" + part.GetValue("name") + "' config node contains KM_Gimbal_3, moving it");
                        }
                }
                ready = true;
            }

            void move_node_first(ConfigNode node, ConfigNode partNode)
            {
                partNode.RemoveNode(node);
                List<ConfigNode> backup_nodes = new List<ConfigNode>();

                for (int i = 0; i < partNode.nodes.Count; i++)
                    backup_nodes.Add(partNode.nodes[i]);
                partNode.nodes.Clear();

                partNode.nodes.Add(node);
                for (int i = 0; i < backup_nodes.Count; i++)
                    partNode.nodes.Add(backup_nodes[i]);
            }
        }
    }
}
