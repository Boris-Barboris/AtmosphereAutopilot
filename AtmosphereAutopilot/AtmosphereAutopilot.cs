/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
Copyright (C) 2016, George Sedov.

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
using System.Reflection;
using KSP.UI.Screens;
using AtmosphereAutopilot.UI;
//using ToolbarWrapper;

namespace AtmosphereAutopilot
{
    [KSPAddon(KSPAddon.Startup.MainMenu, true)]
    public sealed class AtmosphereAutopilot: MonoBehaviour
    {
        // List of all autopilot modules, that need to be created for vessel
        internal List<Type> autopilot_module_types = new List<Type>();

        // Map of vessel - module list relation
        internal Dictionary<Vessel, Dictionary<Type, AutopilotModule>> autopilot_module_lists =
            new Dictionary<Vessel, Dictionary<Type, AutopilotModule>>(new VesselOldComparator());

        // Application launcher window
        AppLauncherWindow applauncher = new AppLauncherWindow();

        // Hotkey manager
        internal AutoHotkey hotkeyManager;

        /// <summary>
        /// Get AtmosphereAutopilot addon class instance
        /// </summary>
        public static AtmosphereAutopilot Instance { get; private set; }

        /// <summary>
        /// Get current active (controlled by player) vessel
        /// </summary>
        public Vessel ActiveVessel { get; private set; }

        private AssetBundle _prefabs;
        internal AssetBundle prefabs {
            get {
                if (_prefabs == null) {
                    var path = System.Reflection.Assembly.GetExecutingAssembly().Location;
                    path = path.Replace (System.IO.Path.GetFileName (path), "atmosphereautopilotprefabs");
                    var www = new WWW("file://"+path);
                    _prefabs = www.assetBundle;
                }
                return _prefabs;
            }
        }

        void Start()
        {
            Debug.Log("[AtmosphereAutopilot]: starting up!"); 
            DontDestroyOnLoad(this);
            determine_aerodynamics();
            get_csurf_module();
            get_gimbal_modules();
            initialize_types();
            hotkeyManager = new AutoHotkey(autopilot_module_types);
            initialize_thread();
            GameEvents.onVesselChange.Add(vesselSwitch);
            GameEvents.onGameSceneLoadRequested.Add(sceneSwitch);
            GameEvents.onHideUI.Add(OnHideUI);
            GameEvents.onShowUI.Add(OnShowUI);
            GameEvents.onGUIApplicationLauncherReady.Add(onAppLauncherLoad);
            GameEvents.onGUIApplicationLauncherUnreadifying.Add(onAppLauncherUnload);
            GameEvents.onGamePause.Add(OnApplicationPause);
            GameEvents.onGameUnpause.Add(OnApplicationUnpause);
            Instance = this;
            ActiveVessel = null;
        }

        public enum AerodinamycsModel
        {
            Stock,
            FAR
        }

        /// <summary>
        /// Current aerodynamics model.
        /// </summary>
        public static AerodinamycsModel AeroModel { get; private set; }
        Assembly far_assembly;

        void determine_aerodynamics()
        {
            AeroModel = AerodinamycsModel.Stock;
            foreach (var a in AppDomain.CurrentDomain.GetAssemblies())
            {
                if (a.GetName().Name.Equals("FerramAerospaceResearch"))
                {
                    far_assembly = a;
                    AeroModel = AerodinamycsModel.FAR;
                    Debug.Log("[AtmosphereAutopilot]: FAR aerodynamics detected");
                    return;
                }
            }
        }

        /// <summary>
        /// Type of module of control surface.
        /// </summary>
        public static Type control_surface_module_type { get; private set; }

        void get_csurf_module()
        {
            if (AeroModel == AerodinamycsModel.Stock)
                control_surface_module_type = typeof(SyncModuleControlSurface);
            else
            {
                control_surface_module_type = far_assembly.GetTypes().First(t => t.Name.Equals("FARControllableSurface"));
                if (control_surface_module_type == null)
                    throw new Exception("AtmosphereAutopilot could not bind to FAR FARControllableSurface class");
            }
        }

        public static Dictionary<Type, ConstructorInfo> gimbal_module_wrapper_map = new Dictionary<Type, ConstructorInfo>(4);

        void get_gimbal_modules()
        {
            gimbal_module_wrapper_map.Add(typeof(ModuleGimbal), typeof(StockGimbal).GetConstructors()[0]);
            if (kmGimbal.do_reflections())
                gimbal_module_wrapper_map.Add(kmGimbal.gimbal_type, typeof(kmGimbal).GetConstructors()[0]);
        }

        void initialize_types()
        {
            // Find all sealed children of AutopilotModule and treat them as complete autopilot modules
            foreach (var asmbly in AppDomain.CurrentDomain.GetAssemblies())
            {
                try
                {
                    var lListOfBs = (from lType in asmbly.GetTypes()
                                     where lType.IsSubclassOf(typeof(AutopilotModule))
                                     where lType.IsSealed
                                     select lType);
                    autopilot_module_types.AddRange(lListOfBs);
                }
                catch (Exception)
                {
                    Debug.Log("[AtmosphereAutopilot]: reflection crash on " + asmbly.FullName + " assembly.");
                }
            }
        }

        // Thread for computation-heavy tasks
        BackgroundThread thread;

        /// <summary>
        /// Get background thread, used by autopilot framework
        /// </summary>
        public BackgroundThread BackgroundThread { get { return thread; } }
        
        void initialize_thread()
        {
            thread = new BackgroundThread(KSPUtil.ApplicationRootPath + "GameData/AtmosphereAutopilot");
        }

        void OnApplicationPause()
        {
            thread.Pause();
        }

        void OnApplicationUnpause()
        {
            thread.Resume();
        }

        void serialize_active_modules()
        {
            if (ActiveVessel == null)
                return;
            foreach (var module in autopilot_module_lists[ActiveVessel].Values)
                module.Serialize();
            hotkeyManager.Serialize();
        }

        void OnDestroy()
        {
            serialize_active_modules();
            AtmosphereAutopilot.Instance.BackgroundThread.Stop();
        }

        void sceneSwitch(GameScenes scenes)
        {
            serialize_active_modules();
            clean_modules();
            if (scenes != GameScenes.FLIGHT)
            {
                mainMenuClose ();
                ActiveVessel = null;
                AtmosphereAutopilot.Instance.BackgroundThread.Pause();
            }
            else
                AtmosphereAutopilot.Instance.BackgroundThread.Resume();
        }

        void load_manager_for_vessel(Vessel v)
        {
            if (v == null)
                return;
            if (!autopilot_module_lists.ContainsKey(v))
            {
                Debug.Log("[AtmosphereAutopilot]: new vessel, creating new module map for " + v.vesselName);
                autopilot_module_lists[v] = new Dictionary<Type, AutopilotModule>();
            }
            if (!autopilot_module_lists[v].ContainsKey(typeof(TopModuleManager)))
                autopilot_module_lists[v][typeof(TopModuleManager)] = new TopModuleManager(v);
			autopilot_module_lists[v][typeof(TopModuleManager)].Deserialize();
            hotkeyManager.Deserialize();
        }

        void vesselSwitch(Vessel v)
        {
            serialize_active_modules();
            Debug.Log("[AtmosphereAutopilot]: vessel switch to " + v.vesselName);
            load_manager_for_vessel(v);
            mainMenuClose ();
            ActiveVessel = v;
            foreach (Vessel c in autopilot_module_lists.Keys)
            {
                if (autopilot_module_lists[c].ContainsKey(typeof(FlightModel)))
                    (autopilot_module_lists[c][typeof(FlightModel)] as FlightModel).sequential_dt = false;
            }
        }

        void clean_modules()
        {
            Debug.Log("[AtmosphereAutopilot]: cleaning modules hash table");
            var vesselsToRemove = autopilot_module_lists.Keys.Where(v => v.state == Vessel.State.DEAD).ToArray();
            foreach (var v in vesselsToRemove)
            {
                var manager = autopilot_module_lists[v][typeof(TopModuleManager)];
                manager.Deactivate();
                autopilot_module_lists.Remove(v);
                Debug.Log("[AtmosphereAutopilot]: removed vessel " + v.vesselName);
                if (autopilot_module_lists.ContainsKey(v))
                    Debug.Log("[AtmosphereAutopilot]: Logical error, not removed from keys");
            }
        }

        /// <summary>
        /// Get the map of AutopilotModule instances, created for arbitrary vessel.
        /// </summary>
        public Dictionary<Type, AutopilotModule> getVesselModules(Vessel v)
        {
            if (autopilot_module_lists.ContainsKey(v))
                return autopilot_module_lists[v];
            else
                return null;
        }


        #region AppLauncherSection

        ApplicationLauncherButton launcher_btn;

        private Texture launcher_btn_textore_off = null;
        private Texture launcher_btn_textore_on = null;

        private UI.MainMenu toolbar_menu = null;
        private GameObject toolbar_menu_object = null;
        private GameObject toolbar_menu_prefab = null;

        // Called when applauncher is ready for population
        void onAppLauncherLoad()
        {
            if (prefabs == null) {
                Debug.Log("[AtmosphereAutopilot]: No prefabs found, GUI unavailable");
                return;
            }
            if (launcher_btn_textore_off == null || launcher_btn_textore_on == null) {
                launcher_btn_textore_off = prefabs.LoadAsset<Texture> ("AA_off");
                launcher_btn_textore_on = prefabs.LoadAsset<Texture> ("AA_on");
            }
            if (ApplicationLauncher.Ready)
            {
                bool hidden;
                bool contains = ApplicationLauncher.Instance.Contains(launcher_btn, out hidden);
                if (!contains)
                    launcher_btn = ApplicationLauncher.Instance.AddModApplication(
                        OnTrue, OnFalse, OnHover, OnUnHover, null, null, 
                        ApplicationLauncher.AppScenes.FLIGHT | ApplicationLauncher.AppScenes.MAPVIEW,
                        launcher_btn_textore_off);
                if (ActiveVessel != null &&
                    autopilot_module_lists.ContainsKey (ActiveVessel) &&
                    autopilot_module_lists[ActiveVessel][typeof(TopModuleManager)] != null)
                      setLauncherOnOffIcon(autopilot_module_lists[ActiveVessel][typeof (TopModuleManager)].Active);
            }
        }

        private void onAppLauncherUnload (GameScenes scene) {
            // remove button
            if (ApplicationLauncher.Instance != null && launcher_btn != null) {
                ApplicationLauncher.Instance.RemoveModApplication(launcher_btn);
                launcher_btn = null;
            }
        }

        private void OnTrue() {
            mainMenuOpen();
        }

        private void OnFalse() {
            mainMenuClose();
        }

        private void OnHover() {
            mainMenuOpen();
        }

        private void OnUnHover() {
            if (launcher_btn != null && launcher_btn.toggleButton.CurrentState == KSP.UI.UIRadioButton.State.False)
                mainMenuClose();
        }

        private void setLauncherOnOffIcon (bool state) {
            if (launcher_btn == null)
                return;
            if (state)
                launcher_btn.SetTexture (launcher_btn_textore_on);
            else 
                launcher_btn.SetTexture (launcher_btn_textore_off);
        }

        private void mainMenuClose() {
            if (toolbar_menu != null)
                toolbar_menu.fadeOut();
            else if (toolbar_menu_object != null)
                Destroy(toolbar_menu_object);
        }

        private void mainMenuOpen() {
          // fade menu in if already open
          if (toolbar_menu != null) {
            toolbar_menu.fadeIn();
            return;
          }

          if (toolbar_menu_prefab == null)
              toolbar_menu_prefab = prefabs.LoadAsset<GameObject> ("AtmosphereAutopilotMainMenu");

          if (toolbar_menu_prefab == null || toolbar_menu_object != null)
            return;

          toolbar_menu_object = Instantiate(toolbar_menu_prefab, GetAnchor(), Quaternion.identity) as GameObject;
          if (toolbar_menu_object == null)
            return;

          toolbar_menu_object.transform.SetParent(MainCanvasUtil.MainCanvas.transform);
          toolbar_menu = toolbar_menu_object.GetComponent<UI.MainMenu> ();
          if (toolbar_menu != null)
            toolbar_menu.setController(new MainMenuController (this));
          GUIStyles.Process(toolbar_menu_object);
        }

        public Vector3 GetAnchor() {
            if (launcher_btn == null)
                return Vector3.zero;

            Vector3 anchor = launcher_btn.GetAnchor();

            anchor.x -= 3.0f;

            return anchor;
        }

        public void mainMenuGUIUpdate () {
            if (toolbar_menu != null)
              toolbar_menu.updateGUI ();
            if (autopilot_module_lists.ContainsKey(ActiveVessel) &&
                autopilot_module_lists[ActiveVessel].ContainsKey(typeof(TopModuleManager)))
              setLauncherOnOffIcon (autopilot_module_lists[ActiveVessel][typeof(TopModuleManager)].Active);
        }

    private class MainMenuController : UI.IGUIController {
      private AtmosphereAutopilot parent;

      private TopModuleManager masterAP = null;
      private StandardFlyByWire fbwAP = null;
      private CruiseController ccAP = null;
      private MouseDirector dcAP = null;
      private ProgradeThrustController speedAP = null;
      private PitchAngularVelocityController pvc = null;
      private YawAngularVelocityController yvc = null;

      private Autopilots currentAP {
        get {
          findModules ();
          if (masterAP != null && masterAP.Active == false)
            return Autopilots.DISABLED;
          if (fbwAP != null && fbwAP.Active)
            return Autopilots.FLYBYWIRE;
          if (ccAP != null && ccAP.Active)
            return Autopilots.CRUISECTRL;
          if (dcAP != null && dcAP.Active)
            return Autopilots.MOUSEDIR;
          return Autopilots.DISABLED;
        }
      }

      private void findModules () {
        if (parent.ActiveVessel == null || parent.autopilot_module_lists.ContainsKey (parent.ActiveVessel) == false)
          return;

        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (TopModuleManager)))
          masterAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (TopModuleManager)] as TopModuleManager;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (StandardFlyByWire)))
          fbwAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (StandardFlyByWire)] as StandardFlyByWire;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (CruiseController)))
          ccAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (CruiseController)] as CruiseController;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (MouseDirector)))
          dcAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (MouseDirector)] as MouseDirector;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (ProgradeThrustController)))
          speedAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (ProgradeThrustController)] as ProgradeThrustController;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (PitchAngularVelocityController)))
          pvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (PitchAngularVelocityController)] as PitchAngularVelocityController;
        if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (YawAngularVelocityController)))
          yvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (YawAngularVelocityController)] as YawAngularVelocityController;
      }

      internal MainMenuController (AtmosphereAutopilot parent) {
        this.parent = parent;

        findModules ();
      }

      public bool appButtonOn {
        get {
          return parent.launcher_btn != null && parent.launcher_btn.toggleButton.CurrentState == KSP.UI.UIRadioButton.State.True;
        }
      }

      public Autopilots currentAutopilot {
        get {
          return currentAP;
        }

        set {
          if (masterAP == null)
            return;

          switch (value) {
            case Autopilots.DISABLED:
              fbwAP = null;
              ccAP = null;
              dcAP = null;
              speedAP = null;
              pvc = null;
              yvc = null;
              masterAP.Active = false;
              parent.setLauncherOnOffIcon (false);
              return;
            case Autopilots.FLYBYWIRE:
              fbwAP = masterAP.activateAutopilot (typeof (StandardFlyByWire)) as StandardFlyByWire;
              ccAP = null;
              dcAP = null;
              break;
            case Autopilots.CRUISECTRL:
              fbwAP = null;
              ccAP = masterAP.activateAutopilot (typeof (CruiseController)) as CruiseController;
              dcAP = null;
              break;
            case Autopilots.MOUSEDIR:
              fbwAP = null;
              ccAP = null;
              dcAP = masterAP.activateAutopilot (typeof (MouseDirector)) as MouseDirector;
              break;
          }
          if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (ProgradeThrustController)))
            speedAP = parent.autopilot_module_lists[parent.ActiveVessel][typeof (ProgradeThrustController)] as ProgradeThrustController;
          if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (PitchAngularVelocityController)))
            pvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (PitchAngularVelocityController)] as PitchAngularVelocityController;
          if (parent.autopilot_module_lists[parent.ActiveVessel].ContainsKey (typeof (YawAngularVelocityController)))
            yvc = parent.autopilot_module_lists[parent.ActiveVessel][typeof (YawAngularVelocityController)] as YawAngularVelocityController;

          parent.setLauncherOnOffIcon (masterAP.Active);
        }
      }

      public bool speedControl {
        get {
          return speedAP != null && speedAP.spd_control_enaled;
        }

        set {
          if (speedAP != null)
            speedAP.spd_control_enaled = value;
        }
      }

      public float speed {
        get {
          if (speedAP == null)
            return 0f;
          return speedAP.spd_setpoint;
        }

        set {
          if (parent.ActiveVessel != null)
            speedAP.spd_setpoint = value;
        }
      }

      // fly-by-wire
      public bool rocketMode {
        get {
          return fbwAP != null && fbwAP.rocket_mode;
        }
        set {
          if (fbwAP != null)
            fbwAP.rocket_mode = value;
        }
      }

      // the "normal" value for all the limits, hardcoded
      private const float moderationNorm = 15f;

      private void setModerationLimits (float limit) {
        if (pvc != null) {
          pvc.max_aoa = limit;
          pvc.max_g_force = limit;
        }
        if (yvc != null) {
          yvc.max_aoa = limit;
          yvc.max_g_force = limit;
        }
      }

      public bool moderation {
        get {
          return fbwAP != null && fbwAP.moderation_switch;
        }
        set {
          if (fbwAP != null)
            fbwAP.moderation_switch = value;
        }
      }

      public float moderationMult {
        get {
          if (pvc == null)
            return 1f;
          //take pitch aoa limit as the master
          float value = pvc.max_aoa;
          setModerationLimits (value);
          return value / moderationNorm;
        }
        set {
          setModerationLimits (value * moderationNorm);
        }
      }

      // cruise control
      public bool altitudeControl {
        get {
          return ccAP != null && ccAP.vertical_control;
        }
        set {
          if (ccAP != null)
            ccAP.vertical_control = value;
        }
      }

      public float altitude {
        get {
          if (ccAP == null)
            return 0f;
          return ccAP.desired_altitude;
        }
        set {
          if (ccAP != null)
            ccAP.desired_altitude = value;
        }
      }

      public float distToWaypoint {
        get {
          if (ccAP == null || ccAP.current_mode != CruiseController.CruiseMode.Waypoint)
            return -1f;

          double straightDist = ccAP.dist_to_dest;
          // no corrections if the target is near
          if (straightDist <= 10000.0)
            return (float)straightDist;
          // calculate the surface distance
          double radius = parent.ActiveVessel.mainBody.Radius;
          return (float)(Math.Acos (1 - (straightDist * straightDist) / (2 * radius * radius)) * radius);
        }
      }

      public bool pickingWaypoint {
        get {
          return ccAP != null && ccAP.current_mode == CruiseController.CruiseMode.Waypoint && ccAP.picking_waypoint;
        }
      }

      public bool waypointIsSet {
        get {
          return ccAP != null && ccAP.current_mode == CruiseController.CruiseMode.Waypoint && ccAP.waypoint_entered;
        }
      }

      public void pickWaypoint () {
        if (ccAP == null)
          return;
        ccAP.WaypointMode = true;
        MapView.EnterMapView ();
        ccAP.picking_waypoint = true;
      }

      public void cancelWaypointPick () {
        if (ccAP == null)
          return;
        ccAP.picking_waypoint = false;
      }

      public void removeWaypoint () {
        if (ccAP == null)
          return;
        ccAP.picking_waypoint = false;
        ccAP.LevelFlightMode = true;
      }
    }

        #endregion

        #region UI

        bool styles_init = false;

        void OnGUI()
        {
            if (!styles_init)
            {
                GUIStyles.Init();
                styles_init = true;
            }
            if (ActiveVessel == null)
                return;
            if (!HighLogic.LoadedSceneIsFlight)
                return;            
            GUIStyles.set_colors();
            applauncher.OnGUI();
            foreach (var pair in autopilot_module_lists[ActiveVessel])
                pair.Value.OnGUI();
            hotkeyManager.OnGUI();
            GUIStyles.reset_colors();
        }

        public static bool UIHidden = false;

        void OnHideUI()
        {
            UIHidden = true;
        }

        void OnShowUI()
        {
            UIHidden = false;
        }

        #endregion

        void Update()
        {
            if (!HighLogic.LoadedSceneIsFlight)
                return;
            if (ActiveVessel == null)
                return;
            if (autopilot_module_lists.ContainsKey(ActiveVessel))
            {
                var module_list = autopilot_module_lists[ActiveVessel].Values.ToList();
                foreach (var module in module_list)
                    if (module.Active || module is TopModuleManager)
                        module.OnUpdate();
            }
        }
    }
}
