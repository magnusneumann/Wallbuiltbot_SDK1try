import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
import numpy as np
import yaml

class ExplorationNavigator(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # --- INPUTS ---
        self.ist_pose = sr.CartesianPose("ist_pose", "base_link")
        self.add_input("_ist_pose", "ist_pose", sr.CartesianPose)

        self.target_pose_in = sr.CartesianPose("target_pose", "base_link")
        self.add_input("_target_pose_in", "target_pose_in", sr.CartesianPose)

        self.take_img_in = False
        self.add_input("_take_img_in", "take_img_in", bool)

        self.img_taken = False
        self.add_input("_img_taken", "img_taken", bool)

        # --- OUTPUTS ---
        self.target_pose_out = sr.CartesianPose("target_pose", "base_link")
        self.add_output("_target_pose_out", "target_pose_out", sr.CartesianPose)

        self.take_img_out = False
        self.add_output("_take_img_out", "take_img_out", bool)

        self.trajectory_success = False
        self.add_output("_trajectory_success", "trajectory_success", bool)

        self.trigger_ppl = False
        self.add_output("_trigger_ppl", "trigger_ppl", bool)

        # --- INTERNE VARIABLEN ---
        self.mode = "EXPLORATION" # Wechselt später auf "GATEWAY"
        self.exploration_pose_list = []
        self.position_tolerance = 0.005 # 5mm Toleranz für "Angekommen"
        self.waiting_for_camera_reset = False # Verhindert Doppelt-Trigger
        
        # ROS-Parameter für den Pfad zur YAML-Datei hinzufügen
        self.yaml_path = "/home/ros2/ws/src/kloetzchenpackagetrynrone/config/ExplCoords.yaml"
        self.add_parameter("_yaml_path", "yaml_path", self.yaml_path)

    def on_setup(self):
        """Wird beim Aktivieren des Blocks einmalig aufgerufen."""
        self._load_yaml_poses()
        return True

    def on_step_callback(self):
        # --- PHASE 1: EXPLORATION ---
        if self.mode == "EXPLORATION":
            
            if len(self.exploration_pose_list) > 0:
                current_target = self.exploration_pose_list[0]
                self.target_pose_out = current_target
                
                # 1. Prüfen, ob wir angekommen sind
                dist = self._pose_distance(self.ist_pose, current_target)
                
                # Wenn wir am Ziel sind UND die Kamera gerade nicht blockiert ist
                if dist <= self.position_tolerance and not self.waiting_for_camera_reset:
                    self.trajectory_success = True
                    self.take_img_out = True # Trigger die Kamera!
                    
                    # 2. Handshake: Warten auf Bestätigung der Kamera
                    if self.img_taken:
                        # Bild ist im Kasten!
                        self.exploration_pose_list.pop(0) # Nächste Pose
                        self.take_img_out = False # Trigger wegnehmen
                        self.trajectory_success = False # Sicherheitshalber resetten
                        self.waiting_for_camera_reset = True # Blockieren, bis Kamera quittiert
                else:
                    self.trajectory_success = False
                
                # 3. Handshake Reset: Warten, bis Kamera wieder bereit ist
                if self.waiting_for_camera_reset and not self.img_taken:
                    self.waiting_for_camera_reset = False
                    
            else:
                # Liste ist leer -> Exploration fertig!
                self.mode = "GATEWAY"
                self.trigger_ppl = True
                self.get_logger().info("Exploration beendet. Schalte in Gateway-Modus.")

        # --- PHASE 2: GATEWAY ---
        elif self.mode == "GATEWAY":
            # Leite Signale einfach 1:1 durch
            self.target_pose_out = self.target_pose_in
            self.take_img_out = self.take_img_in
            
            # Trajectory Success berechnen wir trotzdem weiterhin aus!
            if not self.target_pose_in.is_empty():
                dist = self._pose_distance(self.ist_pose, self.target_pose_in)
                self.trajectory_success = (dist <= self.position_tolerance)

    # --- HILFSFUNKTIONEN ---
    def _pose_distance(self, pose1: sr.CartesianPose, pose2: sr.CartesianPose):
        """Berechnet die euklidische Distanz zwischen zwei Posen (nur X,Y,Z)."""
        if pose1.is_empty() or pose2.is_empty():
            return 999.0
        p1 = pose1.get_position()
        p2 = pose2.get_position()
        return np.linalg.norm(p1 - p2)

    def _load_yaml_poses(self):
        """Lädt die Posen aus der YAML Datei und wandelt sie in CartesianPoses um."""
        try:
            with open(self.yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                # Annahme: YAML enthält eine Liste von Dictionaries mit x,y,z,qx,qy,qz,qw
                for item in data.get('poses', []):
                    pose = sr.CartesianPose("expl_pose", "base_link")
                    pose.set_position([item['x'], item['y'], item['z']])
                    pose.set_orientation([item['qw'], item['qx'], item['qy'], item['qz']])
                    self.exploration_pose_list.append(pose)
            self.get_logger().info(f"{len(self.exploration_pose_list)} Posen geladen.")
        except Exception as e:
            self.get_logger().error(f"Fehler beim Laden der YAML: {e}")