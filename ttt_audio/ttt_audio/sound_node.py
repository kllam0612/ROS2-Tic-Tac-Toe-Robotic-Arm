import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from ttt_interfaces.action import TextToSpeech
import subprocess
import time
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory

class SoundNode(Node):
    def __init__(self):
        super().__init__('sound_node')
        
        # 1. Internal state for Single Goal Enforcement
        self._goal_handle = None
        self._goal_lock = False
        
        # 1.1 Declare ROS 2 Parameters with defaults
        self.declare_parameter('voice_name', 'en+m1')
        self.declare_parameter('speech_speed', 150)
        self.declare_parameter('speech_pitch', 50)

        # 2. Load Configuration (YAML)
        # Initialize defaults before loading
        self.language = 'en' 
        self.voice_name = 'en+m1'
        self.speech_speed = 150
        self.speech_pitch = 50
        
        self.load_config()

        # 3. Create Action Server
        self._action_server = ActionServer(
            self,
            TextToSpeech,
            'text_to_speech',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback
        )
        
        self.get_logger().info(f"Sound Node Ready.")

    def load_config(self):
        """
        Loads parameters from two YAML sources:
        1. yahboom.yaml -> Determines base 'language' (en/zh)
        2. sound_config.yaml -> Determines specific voice/pitch/speed
        """
        
        # --- 1. Load Base Language from Yahboom Config ---
        yahboom_path = os.path.join(
            get_package_share_directory('ttt_audio'),
            'config',
            'yahboom.yaml'
        )

        try:
            with open(yahboom_path, 'r') as file:
                config = yaml.safe_load(file)
                # Check for 'action_service' key (handles potential leading spaces in YAML)
                if config and 'action_service' in config:
                    ros_params = config['action_service'].get('ros__parameters', {})
                    self.language = ros_params.get('language', 'en')
                    #self.get_logger().info(f"Base Language loaded from Yahboom: {self.language}")
        except Exception as e:
            self.get_logger().warn(f"Yahboom config load failed ({e}). Defaulting language to 'en'.")

        # Set a default voice based on the loaded language
        # This serves as a fallback if sound_config.yaml is missing or incomplete
        default_voice = 'zh' if self.language == 'zh' else 'en-us'
        self.voice_name = default_voice

        # --- 2. Load Specific Audio Settings from Sound Config ---
        sound_config_file = os.path.join(
            get_package_share_directory('ttt_audio'),
            'config',
            'sound_config.yaml'
        )

        try:
            with open(sound_config_file, 'r') as file:
                data = yaml.safe_load(file)
                
                # Navigate hierarchy: sound_params -> ros__parameters -> sound_settings
                if data and 'sound_params' in data:
                    settings = data['sound_params']['ros__parameters']['sound_settings']
                    
                    # Update internal variables (Overrides language default)
                    # Use .get() for safety in case a specific key is missing
                    self.voice_name = str(settings.get('voice', self.voice_name))
                    self.speech_speed = int(settings.get('speech_speed', self.speech_speed))
                    self.speech_pitch = int(settings.get('speech_pitch', self.speech_pitch))
                    
                    self.get_logger().info(f"YAML Loaded; Voice: {self.voice_name}, Speed: {self.speech_speed}, Pitch: {self.speech_pitch}")
        except Exception as e:
            self.get_logger().error(f"Failed to load sound_config.yaml: {e}. Using derived defaults.")

        # --- 3. Sync to ROS 2 Parameters ---
        # This ensures get_parameter() in execute_callback returns the YAML values
        self.set_parameters([
            rclpy.parameter.Parameter('voice_name', rclpy.Parameter.Type.STRING, self.voice_name),
            rclpy.parameter.Parameter('speech_speed', rclpy.Parameter.Type.INTEGER, self.speech_speed),
            rclpy.parameter.Parameter('speech_pitch', rclpy.Parameter.Type.INTEGER, self.speech_pitch)
        ])

    def goal_callback(self, goal_request):
        """
        9) Single Goal Enforcement: Rejects new goals if one is already running.
        """
        if self._goal_lock:
            self.get_logger().warn("Rejecting new goal: Sound node is busy speaking.")
            return GoalResponse.REJECT
        
        self.get_logger().info(f"Goal Received: {goal_request.say}")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """
        Locks the node and starts execution.
        """
        self._goal_lock = True
        self._goal_handle = goal_handle
        goal_handle.execute()

    def execute_callback(self, goal_handle):
        """
        10) Execution loop with 10s timeout and process monitoring.
        Now supports dynamic parameter injection for voice, speed, and pitch.
        """
        text = goal_handle.request.say
        feedback_msg = TextToSpeech.Feedback()
        result = TextToSpeech.Result()
        
        # 15) Audio Volume Control
        self.set_volume(50)

        # -- Dynamic Configuration Retrieval --
        voice_param = self.get_parameter('voice_name').value
        speed_param = self.get_parameter('speech_speed').value
        pitch_param = self.get_parameter('speech_pitch').value

        #self.get_logger().info(f"Speaking: '{text}' (Voice: {voice_param}, Speed: {speed_param}, Pitch: {pitch_param})")

        # 11) & 12) & 13) Construct command
        # espeak -> stdout -> aplay (plughw:0,0)
        # Using dynamic parameters instead of hardcoded strings
        
        # Using sh -c to handle the pipe properly in Popen
        cmd = f"espeak -v {voice_param} -s {speed_param} -p {pitch_param} '{text}' --stdout | aplay -D plughw:0,0"
        
        # 14) Start process
        process = subprocess.Popen(
            cmd, 
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid # Allow killing the whole process group
        )

        start_time = time.time()
        timeout = 10.0
        success = False
        aborted = False

        while True:
            # Check if process is finished
            retcode = process.poll()
            
            # 10) Check timeout
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                self.get_logger().error("TTS Timeout: Aborting goal.")
                self.kill_process(process)
                aborted = True
                break

            if retcode is not None:
                # Process finished
                if retcode == 0:
                    success = True
                else:
                    self.get_logger().error(f"TTS Process failed with return code {retcode}")
                break

            # Send feedback
            #feedback_msg.log2 = f"Speaking... ({elapsed_time:.2f}s)"
            goal_handle.publish_feedback(feedback_msg)
            
            # Sleep briefly to prevent CPU hogging
            time.sleep(0.1)

        # Cleanup and finalize
        self._goal_lock = False
        self._goal_handle = None

        if aborted:
            goal_handle.abort()
            result.say_done = False
            return result
        
        if success:
            goal_handle.succeed()
            result.say_done = True
            self.get_logger().info("Speech goal completed successfully.")
        else:
            # If process failed (non-zero exit) but didn't timeout
            goal_handle.abort()
            result.say_done = False
            self.get_logger().error("Speech failed execution.")

        return result

    def set_volume(self, volume_level=100):
        """
        Sets the system mixer volume to ensure audibility.
        """
        try:
            # Setting Master volume or PCM depending on the card. 
            # -c 0 selects card 0.
            subprocess.run(
                ['amixer', '-c', '0', 'set', 'PCM', f'{volume_level}%'], 
                check=False,
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
            subprocess.run(
                ['amixer', '-c', '0', 'set', 'Speaker', f'{volume_level}%'], 
                check=False,
                stdout=subprocess.DEVNULL, 
                stderr=subprocess.DEVNULL
            )
        except Exception as e:
            self.get_logger().warn(f"Could not set volume: {e}")

    def kill_process(self, process):
        """
        Helper to kill the subprocess group.
        """
        try:
            os.killpg(os.getpgid(process.pid), 9) # SIGKILL
        except Exception as e:
            self.get_logger().warn(f"Failed to kill process: {e}")

def main(args=None):
    rclpy.init(args=args)
    sound_node = SoundNode()
    
    try:
        rclpy.spin(sound_node)
    except KeyboardInterrupt:
        pass
    finally:
        sound_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
