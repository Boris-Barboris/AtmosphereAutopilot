using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Text.RegularExpressions;
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
                int partLoaderIdx = 0;
                while (partLoaderIdx < list.Count)
                {
                    if (list[partLoaderIdx].GetType() == typeof(PartLoader))
                    {
                        Debug.Log("[AtmosphereAutopilot]: PartLoader loader found, injecting mod loaders before it");
                        GameObject go = new GameObject("GimbalRearranger");
                        Rearranger ra = go.AddComponent<Rearranger>();
                        CSurfaceReplacer re = go.AddComponent<CSurfaceReplacer>();
                        list.Insert(partLoaderIdx, ra);
                        list.Insert(partLoaderIdx, re);
                        break;
                    }
                    partLoaderIdx++;
                }
                if (partLoaderIdx == list.Count)
                    Debug.LogError("[AtmosphereAutopilot]: PartLoader loader not found, unable to rearrange gimbals");
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
                        if (move_node_first(gimbal_node, part))
                        {
                            handle_ModuleSurfaceFX(part);
                            Debug.Log("[AtmosphereAutopilot]: part '" + part.GetValue("name") + "' config node contains ModuleGimbal, moving it");
                        }
                    }
                    else if ((gimbal_node = part.nodes.GetNode("MODULE", "name", "KM_Gimbal_3")) != null)
                    {
                        if (move_node_first(gimbal_node, part))
                        {
                            handle_ModuleSurfaceFX(part);
                            Debug.Log("[AtmosphereAutopilot]: part '" + part.GetValue("name") + "' config node contains KM_Gimbal_3, moving it");
                        }
                    }
                }
                ready = true;
            }

            bool move_node_first(ConfigNode gimbalNode, ConfigNode partNode)
            {
                int gimbal_index = -1;
                for (int i = 0; i < partNode.nodes.Count; i++)
                    if (gimbalNode == partNode.nodes[i])
                    {
                        gimbal_index = i;
                        break;
                    }
                if (gimbal_index == 0)
                    return false;
                partNode.RemoveNode(gimbalNode);
                List<ConfigNode> backup_nodes = new List<ConfigNode>();

                for (int i = 0; i < partNode.nodes.Count; i++)
                    backup_nodes.Add(partNode.nodes[i]);
                partNode.nodes.Clear();

                partNode.nodes.Add(gimbalNode);
                for (int i = 0; i < backup_nodes.Count; i++)
                    partNode.nodes.Add(backup_nodes[i]);

                if (gimbal_index > 0)
                    return true;
                return false;
            }

            void handle_ModuleSurfaceFX(ConfigNode partNode)
            {
                ConfigNode[] fx_nodes = partNode.nodes.GetNodes("MODULE", "name", "ModuleSurfaceFX");
                if (fx_nodes != null && fx_nodes.Length != 0)
                {
                    foreach (var fx_node in fx_nodes)
                    {
                        int old_index = -1;
                        if (int.TryParse(
                            fx_node.GetValue("thrustProviderModuleIndex"), out old_index))
                        {
                            fx_node.SetValue("thrustProviderModuleIndex", (old_index + 1).ToString());
                        }
                    }
                }
            }
        }


        class CSurfaceReplacer : LoadingSystem
        {
            bool ready = false;

            public override bool IsReady()
            {
                return ready;
            }

            public override void StartLoad()
            {
                // replace ModuleControlSurface modules with SyncModuleControlSurface
                string propellerRegex = @"Propeller|Blade";
                var part_configs = GameDatabase.Instance.GetConfigNodes("PART");
                foreach (var part in part_configs)
                {
                    ConfigNode csurf_node;
                    if ((csurf_node = part.nodes.GetNode("MODULE", "name", "ModuleControlSurface")) != null)
                    {
                        Match m = Regex.Match(part.GetValue("name"), propellerRegex, RegexOptions.IgnoreCase);
                        if (!m.Success)
                        {
                            Debug.Log("[AtmosphereAutopilot]: part '" + part.GetValue("name") + "' config node contains ModuleControlSurface, replacing it");
                            csurf_node.SetValue("name", "SyncModuleControlSurface", false);
                        }
                    }
                }
                ready = true;
            }
        }
    }
}
