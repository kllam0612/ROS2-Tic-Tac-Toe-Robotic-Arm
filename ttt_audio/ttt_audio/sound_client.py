import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ttt_interfaces.action import TextToSpeech
import tkinter as tk
from tkinter import scrolledtext
import threading

class SoundActionClient(Node):
    def __init__(self):
        super().__init__('sound_client')
        self._action_client = ActionClient(self, TextToSpeech, 'text_to_speech')
        self.ui_logger = None  # callback to update UI

    def send_text_goal(self, text):
        """Sends the text goal to the action server."""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.log_to_ui("Server not available!")
            return

        goal_msg = TextToSpeech.Goal()
        goal_msg.say = text

        self.log_to_ui(f"Sending goal: '{text}'")
        
        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle server acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log_to_ui("Goal rejected by server (Is it busy?)")
            return

        self.log_to_ui("Goal accepted, speaking...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle streaming feedback from server."""
        feedback = feedback_msg.feedback
        #self.log_to_ui(f"Feedback: {feedback.log2}")

    def get_result_callback(self, future):
        """Handle final result."""
        result = future.result().result
        status = "Success" if result.say_done else "Failed"
        self.log_to_ui(f"Finished. Status: {status}")
        self.log_to_ui("-" * 30)

    def log_to_ui(self, message):
        """Helper to send logs to the GUI thread safely."""
        if self.ui_logger:
            self.ui_logger(message)
        # Also log to console
        self.get_logger().info(message)


class TTSApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.ros_node.ui_logger = self.update_log_area
        
        self.root.title("Text-To-Speech Tester")
        self.root.geometry("400x350")

        # --- UI Layout ---
        
        # Label
        tk.Label(root, text="Enter text to speak:", font=("Arial", 10, "bold")).pack(pady=5)

        # Input Field
        self.entry_text = tk.Entry(root, width=40)
        self.entry_text.pack(pady=5)
        self.entry_text.insert(0, "Hello world, this is a test.") # Default text

        # Send Button
        self.btn_send = tk.Button(root, text="Speak", command=self.on_send, bg="#4CAF50", fg="white")
        self.btn_send.pack(pady=10, ipadx=20)

        # Log Area (Feedback)
        tk.Label(root, text="Action Logs:", anchor="w").pack(fill="x", padx=10)
        self.log_area = scrolledtext.ScrolledText(root, width=45, height=10, state='disabled')
        self.log_area.pack(pady=5, padx=10)

    def on_send(self):
        text = self.entry_text.get()
        if text.strip():
            # Run the ROS call
            self.ros_node.send_text_goal(text)
        else:
            self.update_log_area("Please enter some text.")

    def update_log_area(self, message):
        """Updates the text area in a thread-safe way."""
        def _update():
            self.log_area.config(state='normal')
            self.log_area.insert(tk.END, message + "\n")
            self.log_area.see(tk.END) # Auto-scroll
            self.log_area.config(state='disabled')
        
        # Schedule update on main UI thread
        self.root.after(0, _update)


def main(args=None):
    rclpy.init(args=args)
    
    # 1. Create the ROS Node
    client_node = SoundActionClient()

    # 2. Run ROS spin in a separate thread so it doesn't block the GUI
    ros_thread = threading.Thread(target=rclpy.spin, args=(client_node,), daemon=True)
    ros_thread.start()

    # 3. Start the GUI
    try:
        root = tk.Tk()
        app = TTSApp(root, client_node)
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        client_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
