import numpy as np
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr
#from sensor_msgs.msg import Image
from state_representation import Image, CartesianPose
class PickPlaceController(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # --- INPUTS ---
        self.trigger_ppl = False
        self.add_input("_trigger_ppl", "trigger_ppl", bool)

        self.trajectory_success = False
        self.add_input("_trajectory_success", "trajectory_success", bool)

        self.img_taken = False
        self.add_input("_img_taken", "img_taken", bool)

        self.filtered_yolo_in = []
        self.add_input("_filtered_yolo_in", "filtered_yolo_in", list)

        self.master_dropoff_in = []
        self.add_input("_master_dropoff_in", "master_dropoff_in", list)

        self.master_overview_in = []
        self.add_input("_master_overview_in", "master_overview_in", list)

        self.depth_image_in = Image()
        self.add_input("_depth_image_in", "depth_image_in", Image)

        # --- OUTPUTS ---
        # Initialisiere die Pose (Name und Referenzsystem - z.B. "base_link")
        self.target_pose_out = sr.CartesianPose("target_pose", "base_link")
        self.add_output("_target_pose_out", "target_pose_out", sr.CartesianPose)

        self.take_img_out = False
        self.add_output("_take_img_out", "take_img_out", bool)

        self.vacuum_on = False
        self.add_output("_vacuum_on", "vacuum_on", bool)

        # --- INTERNE VARIABLEN & STATE MACHINE ---
        self.state = "INIT"
        self.last_traj_success = False
        
        # Interne Listen
        self.overview_list = []
        self.dropoff_list = []
        self.pick_list = []
        self.place_list = []

        # Konstanten (in Metern)
        self.z_hover = 0.15
        self.z_sauger = 0.002
        self.kamera_tcp_abstand = 0.05
        
        # Timer
        self.timer_start = None

    def on_step_callback(self):
        # 1. FLANKENAUSWERTUNG (Edge Detection)
        # Ist True in exakt dem Taktzyklus, in dem der Roboter ankommt.
        traj_success_rising_edge = self.trajectory_success and not self.last_traj_success
        self.last_traj_success = self.trajectory_success

        # 2. STATE MACHINE
        if self.state == "INIT":
            if self.trigger_ppl:
                # Kopiere die Masterlisten für die interne Abarbeitung
                self.dropoff_list = list(self.master_dropoff_in)
                self.overview_list = list(self.master_overview_in)
                self.state = "CHECK_LISTS"

        elif self.state == "CHECK_LISTS":
            if len(self.dropoff_list) > 0 and len(self.overview_list) > 0:
                self.state = "MOVE_OVERVIEW"
            else:
                # Programmende: Sende Home-Position (hardgecodet)
                home_pose = sr.CartesianPose("target_pose", "base_link")
                home_pose.set_position([0.3, 0.0, 0.5]) # Beispiel-Koordinaten
                self.target_pose_out = home_pose
                self.state = "DONE"

        # --- PICK SEQUENZ ---
        elif self.state == "MOVE_OVERVIEW":
            # Wandle Listeneintrag in sr.CartesianPose um (Annahme: Array mit [x,y,z, ...])
            self._set_target_pose(self.overview_list[0])
            
            if traj_success_rising_edge:
                self.take_img_out = True
                self.state = "WAIT_IMG_1"

        elif self.state == "WAIT_IMG_1":
            if self.img_taken:
                self.take_img_out = False # Handshake beendet
                
                if len(self.filtered_yolo_in) == 0:
                    self.overview_list.pop(0)
                    self.state = "CHECK_LISTS"
                else:
                    # Annahme: filtered_yolo_in ist sortiert oder wir sortieren hier
                    # Format: [x, y, mask_size] pro Klotz. Wir nehmen den Ersten.
                    x_grob = self.filtered_yolo_in[0][0]
                    y_grob = self.filtered_yolo_in[0][1]
                    
                    # Tiefenbild auswerten (5x5 Patch)
                    # Achtung: Koordinaten müssen ggf. von Welt in Pixel umgerechnet werden,
                    # falls x_grob/y_grob Weltkoordinaten sind. Hier als Platzhalter:
                    z_pick = self._get_depth_median(self.depth_image_in) 
                    
                    self.pick_list = []
                    # Pose 1 (Hover über Klotz)
                    self.pick_list.append([x_grob, y_grob, z_pick + self.z_hover - self.kamera_tcp_abstand])
                    self.state = "MOVE_PICK_HOVER"

        elif self.state == "MOVE_PICK_HOVER":
            self._set_target_pose(self.pick_list[0])
            if traj_success_rising_edge:
                self.take_img_out = True
                self.state = "WAIT_IMG_2"

        elif self.state == "WAIT_IMG_2":
            if self.img_taken:
                self.take_img_out = False
                
                if len(self.filtered_yolo_in) == 0:
                    # KLOTZ VERLOREN! Aber wir löschen die Overview-Pose NICHT.
                    # Wir brechen den Pick ab und fangen bei dieser Overview-Pose neu an.
                    self.state = "MOVE_OVERVIEW" 
                else:
                    x_fein = self.filtered_yolo_in[0][0]
                    y_fein = self.filtered_yolo_in[0][1]
                    # Z-Berechnung mit Kamera-Versatz
                    z_pick = self.pick_list[0][2] - self.z_hover + self.kamera_tcp_abstand 
                    
                    self.pick_list.append([x_fein, y_fein, z_pick - self.z_sauger])
                    self.pick_list.append([x_fein, y_fein, z_pick + self.z_hover])
                    self.state = "EXECUTE_PICK"

        elif self.state == "EXECUTE_PICK":
            self._set_target_pose(self.pick_list[1])
            if traj_success_rising_edge:
                self.vacuum_on = True
                self.timer_start = self.get_clock().now()
                self.state = "PICK_DELAY"

        elif self.state == "PICK_DELAY":
            # Nicht-blockierender Timer (Warte 0.3 Sekunden)
            elapsed_time = (self.get_clock().now() - self.timer_start).nanoseconds / 1e9
            if elapsed_time >= 0.3:
                self.state = "RETRACT_PICK"

        elif self.state == "RETRACT_PICK":
            self._set_target_pose(self.pick_list[2])
            if traj_success_rising_edge:
                self.pick_list.clear()
                self.state = "PREPARE_PLACE"

        # --- PLACE SEQUENZ ---
        elif self.state == "PREPARE_PLACE":
            x_drop, y_drop, z_drop = self.dropoff_list[0][:3]
            self.place_list = []
            self.place_list.append([x_drop, y_drop, z_drop + self.z_hover]) # Hover
            self.place_list.append([x_drop, y_drop, z_drop - 0.001])        # Drop (1mm tiefer)
            self.place_list.append([x_drop, y_drop, z_drop + self.z_hover]) # Retract
            self.state = "MOVE_PLACE_HOVER"

        elif self.state == "MOVE_PLACE_HOVER":
            self._set_target_pose(self.place_list[0])
            if traj_success_rising_edge:
                self.state = "EXECUTE_PLACE"

        elif self.state == "EXECUTE_PLACE":
            self._set_target_pose(self.place_list[1])
            if traj_success_rising_edge:
                self.vacuum_on = False
                self.timer_start = self.get_clock().now()
                self.state = "PLACE_DELAY"

        elif self.state == "PLACE_DELAY":
            elapsed_time = (self.get_clock().now() - self.timer_start).nanoseconds / 1e9
            if elapsed_time >= 0.5:
                self.state = "RETRACT_PLACE"

        elif self.state == "RETRACT_PLACE":
            self._set_target_pose(self.place_list[2])
            if traj_success_rising_edge:
                self.dropoff_list.pop(0) # Platz ist belegt!
                self.state = "CHECK_LISTS"

        elif self.state == "DONE":
            pass # Arbeit ist getan, warte auf neuen Trigger

    # --- HILFSFUNKTIONEN ---
    def _set_target_pose(self, coords):
        """Hilfsfunktion, um X,Y,Z in eine sr.CartesianPose zu schreiben"""
        pose = CartesianPose("target_pose", "base_link")
        pose.set_position([coords[0], coords[1], coords[2]])
        
        # WICHTIG: Quaternion für "Sauger zeigt nach unten". 
        # (Werte müssen an euer KUKA-Koordinatensystem angepasst werden, z.B. Rx=180°)
        # Format ist meist [x, y, z, w]
        pose.set_orientation([1.0, 0.0, 0.0, 0.0]) 
        
        self.target_pose_out = pose

    def _get_depth_median(self, depth_image_obj):
        """
        Wandelt das Pixel-Zentrum in eine Welt-Z-Koordinate um.
        """
        # 1. Numpy Array aus dem AICA Image Objekt extrahieren
        # (Beispielhaft, genauer Befehl hängt von AICA Version ab)
        depth_array = depth_image_obj.get_data() 
        
        # 2. TODO: Mathematische Transformation
        # Da wir schräg schauen (Overview), muss das Pixel [u,v] mit der Tiefe d 
        # über die Kameramatrix (Intrinsics) und die cam_ist_pose (Extrinsics) 
        # in das base_link-System transformiert werden, um das echte Welt-Z zu erhalten.
        
        # Platzhalter-Rückgabe
        return 0.10 # 10 cm als Dummy