import numpy as np
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr

class MasterListManager(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # --- INPUTS ---
        self.ist_pose_in = sr.CartesianPose("ist_pose", "base_link")
        self.add_input("_ist_pose_in", "ist_pose_in", sr.CartesianPose)

        self.yolo_list_in = []
        # Event: Wenn YOLO neue Klötze schickt
        self.add_input("_yolo_list_in", "yolo_list_in", list, user_callback=self._on_yolo_data)

        self.line_ex_list_in = []
        # Event: Wenn LineExtractor neue Linien schickt
        self.add_input("_line_ex_list_in", "line_ex_list_in", list, user_callback=self._on_line_data)

        # --- OUTPUTS ---
        self.master_dropoff_out = []
        self.add_output("_master_dropoff_out", "master_dropoff_out", list)

        self.master_overview_out = []
        self.add_output("_master_overview_out", "master_overview_out", list)

        self.filtered_yolo_out = []
        self.add_output("_filtered_yolo_out", "filtered_yolo_out", list)

        # --- INTERNE VARIABLEN ---
        self.kollisions_radius = 0.01 # 1 cm in Metern

    def on_step_callback(self):
        pass

    # --- EVENT 1: YOLO Daten verarbeiten ---
    def _on_yolo_data(self):
        # 1. Overview-Liste updaten
        # Wenn YOLO mindestens einen Klotz gefunden hat (Array > 0)
        if len(self.yolo_list_in) > 0:
            # Pose in Liste Format wandeln, damit sie gut transportiert werden kann
            pose_array = self.ist_pose_in.get_position().tolist() # [x, y, z]
            # Wir hängen die Pose an die Master-Overview an
            self.master_overview_out.append(pose_array)
            self.get_logger().info("Pose zur Overview-Liste hinzugefügt.")

        # 2. YOLO Liste filtern (Kollisionsprüfung)
        self._filter_and_publish_yolo()


    # --- EVENT 2: Linien Daten verarbeiten ---
    def _on_line_data(self):
        # Wenn die NEUE Liste mehr Einträge hat als unser bisheriger Masterplan
        # (Annahme: 1 Platz = 3 Einträge [x, y, angle])
        if len(self.line_ex_list_in) > len(self.master_dropoff_out):
            self.master_dropoff_out = list(self.line_ex_list_in) # Kopie speichern
            self.get_logger().info("Neuer, besserer Belegungsplan gespeichert!")
            
            # Da wir jetzt neue Linien haben, müssen wir die YOLO-Liste sicherheitshalber 
            # nochmal filtern, falls ein Klotz auf der NEUEN Linie liegt!
            self._filter_and_publish_yolo()


    # --- HILFSFUNKTION FÜR DEN FILTER ---
    def _filter_and_publish_yolo(self):
        """Löscht Klötze aus der YOLO-Liste, die bereits auf einer Ablagelinie liegen."""
        if len(self.master_dropoff_out) == 0 or len(self.yolo_list_in) == 0:
            self.filtered_yolo_out = list(self.yolo_list_in)
            return

        safe_yolo_list = []
        
        # YOLO-Liste iterieren (Annahme: [x, y, size, angle, ...]) -> 4er Schritte
        for i in range(0, len(self.yolo_list_in), 4):
            klotz_x = self.yolo_list_in[i]
            klotz_y = self.yolo_list_in[i+1]
            
            liegt_auf_linie = False
            
            # Master-Dropoff iterieren (Annahme: [x, y, angle, ...]) -> 3er Schritte
            for j in range(0, len(self.master_dropoff_out), 3):
                drop_x = self.master_dropoff_out[j]
                drop_y = self.master_dropoff_out[j+1]
                
                # Euklidische Distanz (nur X und Y)
                dist = np.sqrt((klotz_x - drop_x)**2 + (klotz_y - drop_y)**2)
                
                if dist <= self.kollisions_radius:
                    liegt_auf_linie = True
                    break # Wir wissen, er ist blockiert, keine weiteren Linien prüfen
                    
            if not liegt_auf_linie:
                # Klotz ist sicher zu greifen! Komplette 4er-Gruppe anhängen
                safe_yolo_list.extend(self.yolo_list_in[i:i+4])
                
        self.filtered_yolo_out = safe_yolo_list