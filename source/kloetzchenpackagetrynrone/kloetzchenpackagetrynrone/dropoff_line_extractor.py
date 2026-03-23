import numpy as np
import cv2
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr

class DropoffLineExtractor(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # --- INPUTS ---
        self.trigger_ppl = False
        self.add_input("_trigger_ppl", "trigger_ppl", bool)

        self.image_in = sr.Image()
        self.add_input("_image_in", "image_in", sr.Image)

        self.cam_ist_pose = sr.CartesianPose("cam_ist_pose", "base_link")
        self.add_input("_cam_ist_pose", "cam_ist_pose", sr.CartesianPose)

        # HIER DER EVENT: Wir reagieren nur, wenn YOLO den Trigger sendet!
        self.yolo_done_trigger = False
        self.add_input("_yolo_done_trigger", "yolo_done_trigger", bool, user_callback=self._on_yolo_trigger)

        # --- OUTPUTS ---
        # Format: [x1, y1, angle1, x2, y2, angle2, ...]
        self.dropoff_coords_list = []
        self.add_output("_dropoff_coords_list", "dropoff_coords_list", list)

        self.debug_img = sr.Image()
        self.add_output("_debug_img", "debug_img", sr.Image)

    def on_step_callback(self):
        pass # Alles läuft event-basiert, der zyklische Step bleibt leer!

    # --- DIE EVENT-FUNKTION ---
    def _on_yolo_trigger(self):
        # 1. KILL-SWITCH PRÜFEN (Sparen von Rechenleistung im PPL-Modus)
        if self.trigger_ppl:
            return 
            
        # 2. FLANKENAUSWERTUNG (Da der Callback bei JEDER Änderung triggert, auch bei True->False)
        if not self.yolo_done_trigger:
            return

        self.get_logger().info("YOLO Trigger empfangen. Starte Linienerkennung...")
        
        # Bild in OpenCV Format holen
        cv_image = self.image_in.get_data()
        
        # --- EURE OPENCV LINIENERKENNUNG (HOUGH, CONTOURS, ETC.) ---
        # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # edges = cv2.Canny(gray, 50, 150)
        # lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
        
        result_list = []
        
        # for line in lines:
        #     x1, y1, x2, y2 = line[0]
        #     u_center = (x1 + x2) / 2
        #     v_center = (y1 + y2) / 2
        #
        #     # WICHTIGES TO-DO: Orientierung berechnen!
        #     # Winkel der Linie im Bild berechnen (in Radian oder Grad)
        #     angle = np.arctan2((y2 - y1), (x2 - x1))
        #
        #     # Transformation der Pixel-Mitte in Weltkoordinaten
        #     world_x, world_y = self._pixel_to_world_2d(u_center, v_center, self.cam_ist_pose)
        #
        #     result_list.extend([world_x, world_y, float(angle)])
        # -------------------------------------------------------------
        
        # Dummy-Daten für jetzt (Zwei freie Plätze)
        result_list = [0.4, -0.2, 1.57,  0.45, -0.2, 1.57] 
        
        # 3. Outputs beschreiben (AICA publisht das danach automatisch)
        self.dropoff_coords_list = result_list
        self.debug_img = sr.Image(self.image_in) # Dummy: Originalbild
        
        self.get_logger().info(f"Linienerkennung fertig. {len(result_list)//3} Plätze gefunden.")

    def _pixel_to_world_2d(self, u, v, cam_pose: sr.CartesianPose):
        """Platzhalter für die Kamera-Transformation (gleich wie im Yolo-Block)."""
        return (0.0, 0.0)