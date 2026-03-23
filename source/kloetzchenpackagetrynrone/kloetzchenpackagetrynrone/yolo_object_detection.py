import numpy as np
import cv2
# from ultralytics import YOLO # Später einkommentieren!
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr

class YoloObjectDetector(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # --- INPUTS ---
        # HIER PASSIERT DIE MAGIE: Wir binden eine Funktion (user_callback) direkt an das Bild!
        self.image_in = sr.Image()
        self.add_input("_image_in", "image_in", sr.Image, user_callback=self._on_new_image)

        self.ist_pose_in = sr.CartesianPose("ist_pose", "base_link")
        self.add_input("_ist_pose_in", "ist_pose_in", sr.CartesianPose)

        self.cam_ist_pose_in = sr.CartesianPose("cam_ist_pose", "base_link")
        self.add_input("_cam_ist_pose_in", "cam_ist_pose_in", sr.CartesianPose)

        # --- OUTPUTS ---
        self.yolo_list = [] # Format: [x1, y1, size1, angle1, x2, y2, size2, angle2, ...]
        self.add_output("_yolo_list", "yolo_list", list)

        self.debug_img = sr.Image()
        self.add_output("_debug_img", "debug_img", sr.Image)

        self.ist_pose_out = sr.CartesianPose("ist_pose_out", "base_link")
        self.add_output("_ist_pose_out", "ist_pose_out", sr.CartesianPose)

        self.yolo_done_trigger = False
        self.add_output("_yolo_done_trigger", "yolo_done_trigger", bool)

        # --- INTERNE VARIABLEN ---
        self.model = None # Platzhalter für das YOLO-Modell
        
        # Kamera-Parameter (Platzhalter - müsst ihr mit eurer Kalibrierung füllen)
        self.camera_matrix = np.array([[1000.0, 0.0, 320.0], 
                                       [0.0, 1000.0, 240.0], 
                                       [0.0, 0.0, 1.0]])

    def on_setup(self):
        """Wird beim Starten des Blocks aufgerufen. Perfekt, um die KI in den RAM zu laden."""
        self.get_logger().info("Lade YOLO Modell...")
        # self.model = YOLO('/pfad/zu/deinem/best_model.pt')
        self.get_logger().info("YOLO Modell geladen!")
        return True

    def on_step_callback(self):
        """
        Da die Bildverarbeitung event-basiert läuft, müssen wir hier nur 
        den Trigger-Impuls nach einem Takt wieder auf False setzen.
        """
        if self.yolo_done_trigger:
            # Im letzten Takt wurde der Trigger auf True gesetzt und von AICA verschickt.
            # Jetzt nehmen wir ihn direkt wieder weg -> Ein perfekter, kurzer Impuls!
            self.yolo_done_trigger = False

    # --- DIE EVENT-FUNKTION (Wird NUR aufgerufen, wenn ein neues Bild ankommt) ---
    def _on_new_image(self):
        self.get_logger().info("Neues Bild empfangen! Starte YOLO...")
        
        # 1. Bild in bearbeitbares Format umwandeln (OpenCV/Numpy)
        # (Genaue AICA-Funktion abhängig von Version, meist .get_data() oder .data)
        cv_image = self.image_in.get_data() 
        
        # --- HIER KOMMT EURE ECHTE YOLO-LOGIK HIN ---
        # results = self.model(cv_image)
        # result_list = []
        # for box in results[0].boxes:
        #     u, v = box.xywh[0][:2]  # Pixel-Zentrum
        #     size = box.xywh[0][2] * box.xywh[0][3] # Bounding Box Fläche (für PPL zum Sortieren)
        #     angle = 0.0 # Später: Winkel für smartes Greifen
        #
        #     # 2. Pixel in 2D-Weltkoordinaten umrechnen
        #     world_x, world_y = self._pixel_to_world_2d(u, v, self.cam_ist_pose_in)
        #
        #     result_list.extend([world_x, world_y, float(size), float(angle)])
        # ---------------------------------------------
        
        # DUMMY DATEN FÜR JETZT:
        result_list = [0.35, -0.1, 1500.0, 0.0] # Ein Klotz bei X=0.35, Y=-0.1
        
        # 3. Outputs beschreiben
        self.yolo_list = result_list
        self.ist_pose_out = sr.CartesianPose(self.ist_pose_in) # Die Roboterpose für DataHandler durchreichen
        
        # Debug-Bild speichern (Hier würdest du das Bild mit Boxen aus YOLO übergeben)
        # self.debug_img = sr.Image(results[0].plot()) 
        self.debug_img = sr.Image(self.image_in) # Dummy: Originalbild durchreichen
        
        # 4. Den Impuls für den nächsten Block zünden!
        self.yolo_done_trigger = True
        self.get_logger().info("YOLO fertig. Trigger gesendet.")

    # --- HILFSFUNKTIONEN ---
    def _pixel_to_world_2d(self, u, v, cam_pose: sr.CartesianPose):
        """
        Transformiert ein 2D-Pixel in 2D-Weltkoordinaten (Z=0 auf dem Tisch).
        """
        # HIER FEHLT NOCH EURE MATHE:
        # 1. Intrinsics: Pixel (u,v) in normalisierte Kamera-Koordinaten umrechnen
        # 2. Extrinsics: Mit cam_pose ins Welt-System rotieren/verschieben
        # 3. Strahlensatz: Den Vektor mit der Tischebene (Z=0) schneiden
        
        # Dummy-Rückgabe
        return (0.0, 0.0)