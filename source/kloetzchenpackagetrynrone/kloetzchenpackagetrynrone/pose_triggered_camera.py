import state_representation as sr
from modulo_components.lifecycle_component import LifecycleComponent
from modulo_components.lifecycle_component import LifecycleComponent
import state_representation as sr

class PoseTriggeredCamera(LifecycleComponent):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        # --- INPUTS ---
        self.take_img = False
        self.add_input("_take_img", "take_img", bool)

        self.trajectory_success = False
        self.add_input("_trajectory_success", "trajectory_success", bool)

        self.ist_pose_in = sr.CartesianPose("ist_pose", "base_link")
        self.add_input("_ist_pose_in", "ist_pose_in", sr.CartesianPose)

        self.cam_ist_pose_in = sr.CartesianPose("cam_ist_pose", "base_link") # Ggf. "camera_link"
        self.add_input("_cam_ist_pose_in", "cam_ist_pose_in", sr.CartesianPose)

        self.image_stream = sr.Image()
        self.add_input("_image_stream", "image_stream", sr.Image)

        # --- OUTPUTS ---
        self.img_taken = False
        self.add_output("_img_taken", "img_taken", bool)

        self.ist_pose_out = sr.CartesianPose("ist_pose_out", "base_link")
        self.add_output("_ist_pose_out", "ist_pose_out", sr.CartesianPose)

        self.cam_ist_pose_out = sr.CartesianPose("cam_ist_pose_out", "base_link")
        self.add_output("_cam_ist_pose_out", "cam_ist_pose_out", sr.CartesianPose)

        self.image_out = sr.Image()
        self.add_output("_image_out", "image_out", sr.Image)

        # --- INTERNE VARIABLEN FÜR TIMER ---
        self.is_delaying = False
        self.delay_start_time = None
        self.delay_duration = 0.3 # in Sekunden

    def on_step_callback(self):
        # 1. HANDSHAKE RESET
        # Sobald der Roboter die Pose verlässt, machen wir uns wieder scharf
        if not self.trajectory_success and self.img_taken:
            self.img_taken = False
            self.is_delaying = False # Sicherheitshalber Timer resetten
            
        # 2. TRIGGER-BEDINGUNG PRÜFEN
        if self.take_img and self.trajectory_success and not self.img_taken:
            
            # 2a. Timer starten (steigt in diese If-Bedingung nur beim allerersten Takt ein)
            if not self.is_delaying:
                self.is_delaying = True
                self.delay_start_time = self.get_clock().now()
                
            # 2b. Timer überwachen (steigt in diese Bedingung in den folgenden Takten ein)
            else:
                elapsed_time = (self.get_clock().now() - self.delay_start_time).nanoseconds / 1e9
                
                # Wenn die 0.3 Sekunden abgelaufen sind -> SNAPSHOT!
                if elapsed_time >= self.delay_duration:
                    self._freeze_and_capture()
                    
                    self.img_taken = True
                    self.is_delaying = False # Timer für den nächsten Zyklus ausschalten
                    
        # 3. SICHERHEITS-ABBRUCH
        # Falls während der 0.3 Sekunden Wartezeit das take_img oder traj_success
        # plötzlich wieder abfällt (z.B. Not-Halt), brechen wir den Timer ab.
        elif self.is_delaying and (not self.take_img or not self.trajectory_success):
            self.is_delaying = False

    # --- HILFSFUNKTIONEN ---
    def _freeze_and_capture(self):
        """Kopiert den aktuellen Zustand der Inputs hart auf die Outputs."""
        
        # In state_representation erstellt die Initialisierung mit dem Objekt als Argument
        # eine echte, tiefe Kopie (Copy Constructor). Das friert den Zustand exakt ein.
        
        # 1. Bild kopieren
        self.image_out = sr.Image(self.image_stream)
        
        # 2. Posen kopieren
        self.ist_pose_out = sr.CartesianPose(self.ist_pose_in)
        self.cam_ist_pose_out = sr.CartesianPose(self.cam_ist_pose_in)
        
        self.get_logger().info("Snapshot erfolgreich ausgelöst und Posen eingefroren.")