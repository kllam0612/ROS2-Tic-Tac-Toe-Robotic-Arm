#!/usr/bin/env python3
import sys
import select
import rclpy
from rclpy.node import Node
from ttt_interfaces.srv import GetHumanMove, ManualUpdate
from ttt_interfaces.msg import BoardState

class UserInterfaceNode(Node):
    def __init__(self):
        super().__init__('ui_node')
        
        # Create Service Servers
        self.srv_get_human_move = self.create_service(
            GetHumanMove, 
            'get_human_move', 
            self.handle_get_human_move
        )
        
        self.srv_manual_update = self.create_service(
            ManualUpdate, 
            'manual_update', 
            self.handle_manual_update
        )

        # Input Mapping: Keypad (1-9) <-> Array Index (0-8)
        # Spec: Physical Keypad Mapping (Cell 9 = Top-Left)
        self.key_to_index = {
            9: 0, 8: 1, 7: 2,
            6: 3, 5: 4, 4: 5,
            3: 6, 2: 7, 1: 8
        }
        # Reverse mapping for logic/display
        self.index_to_key = {v: k for k, v in self.key_to_index.items()}

        self.get_logger().info("User Interface Node Ready.")

    # -------------------------------------------------------------------------
    # 12) Hardware Interface & Helper Methods
    # -------------------------------------------------------------------------
    def get_input_safe(self, prompt_text):
        """
        Reads input from stdin using select() to allow ROS shutdown (Ctrl+C).
        Non-blocking loop mechanism.
        """
        print(prompt_text, end='', flush=True)
        
        while rclpy.ok():
            # System Timeout 0.1s
            # Checks if sys.stdin has data waiting
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            
            if rlist:
                # 12c) Input available, read it
                line = sys.stdin.readline()
                return line.strip()
            
            # No input, loop yields to allow checking rclpy.ok() again
            
        return None  # Return None if ROS shuts down

    def print_board(self, cells):
        """
        Visualizes the board state.
        Mapping: Index 0 = Top-Left, Key 9 = Top-Left
        """
        print("\n--- Current Board ---")
        row1 = f"[{cells[0]}] [{cells[1]}] [{cells[2]}]"
        row2 = f"[{cells[3]}] [{cells[4]}] [{cells[5]}]"
        row3 = f"[{cells[6]}] [{cells[7]}] [{cells[8]}]"
        
        print(f"{row1}   (Keys: 9 8 7)")
        print(f"{row2}   (Keys: 6 5 4)")
        print(f"{row3}   (Keys: 3 2 1)")
        print("---------------------")

    # -------------------------------------------------------------------------
    # 10) Handle Get Human Move
    # -------------------------------------------------------------------------
    def handle_get_human_move(self, request, response):
        current_cells = request.state.cells
        self.print_board(current_cells)

        # Identify available keys based on empty cells (0)
        available_keys = []
        for index, val in enumerate(current_cells):
            if val == 0:
                available_keys.append(self.index_to_key[index])
        
        available_keys.sort()
        
        valid_input = False
        selected_key = -1

        # Loop until valid input or Shutdown
        while not valid_input and rclpy.ok():
            prompt = f"Select Cell {available_keys}: "
            user_input = self.get_input_safe(prompt)
            
            if user_input is None: 
                break # ROS Shutdown triggered

            try:
                key = int(user_input)
                if key in available_keys:
                    selected_key = key
                    valid_input = True
                else:
                    print(f"Error: Key {key} is not available. Try again.")
            except ValueError:
                print("Error: Invalid input. Please enter a number.")

        # Return human input as selected_cell (1-9)
        # Explicit cast (though selected_key is already int, good practice)
        response.target_cell = int(selected_key)
        return response

    # -------------------------------------------------------------------------
    # 11) Handle Manual Update
    # -------------------------------------------------------------------------
    def handle_manual_update(self, request, response):
        # 11a) Load state locally
        # 18) Explicit cast to standard Python int to prevent serialization errors
        working_cells = [int(x) for x in request.state.cells]
        
        updating = True
        print("\n=== MANUAL UPDATE MODE ===")
        print("Cycle Order: 0(Empty) -> 1(Robot) -> 2(Human) -> 0(Empty)")

        # 11b) Toggle Loop
        while updating and rclpy.ok():
            self.print_board(working_cells)
            user_input = self.get_input_safe("Enter Cell (1-9) to toggle, 'd' to Done: ")

            if user_input is None: 
                break # ROS Shutdown triggered

            if user_input.lower() == 'd':
                updating = False
                continue

            try:
                key = int(user_input)
                if key in self.key_to_index:
                    idx = self.key_to_index[key]
                    
                    # Toggle Logic: 0->1->2->0
                    current_val = working_cells[idx]
                    if current_val == 0:
                        working_cells[idx] = 1 # Robot
                    elif current_val == 1:
                        working_cells[idx] = 2 # Human
                    else:
                        working_cells[idx] = 0 # Empty
                else:
                    print("Invalid Key. Use 1-9.")
            except ValueError:
                print("Invalid Input.")

        if not rclpy.ok():
            return response
        
        # ---------------------------------------------------------------
        # Initialize from BoardState.msg
        # ---------------------------------------------------------------
        final_last_move_cell = request.state.last_move_cell
        final_last_move_player = request.state.last_move_player

        print(f"\n--- Metadata Update (Enter to keep [Current]) ---")

        # Ask for Last Move Cell
        valid_meta = False
        while not valid_meta and rclpy.ok():
            prompt = f"Enter Last Move Cell (-1/1-9) [Current: {final_last_move_cell}]: "
            txt = self.get_input_safe(prompt)
            if txt is None: break
            
            if txt == "":
                # Keep the value read from BoardState.msg
                valid_meta = True
            else:
                try:
                    val = int(txt)
                    if 1 <= val <= 9 or val == -1:
                        final_last_move_cell = val
                        valid_meta = True
                    else:
                        print("Please enter -1/1-9.")
                except ValueError:
                    print("Invalid number.")

        # Ask for Last Move Player
        valid_meta = False
        while not valid_meta and rclpy.ok():
            prompt = f"Enter Last Move Player (-1/1/2) [Current: {final_last_move_player}]: "
            txt = self.get_input_safe(prompt)
            if txt is None: break
            
            if txt == "":
                # Keep the value read from BoardState.msg
                valid_meta = True
            else:
                try:
                    val = int(txt)
                    if val in [-1, 1, 2]:
                        final_last_move_player = val
                        valid_meta = True
                    else:
                        print("Please enter -1/1/2.")
                except ValueError:
                    print("Invalid number.")

        # 11d) Display & Return
        print("\n=== UPDATE COMMITTED ===")
        self.print_board(working_cells)
        print(f"Last Move Cell: {final_last_move_cell}")
        print(f"Last Move Player: {final_last_move_player}")

        new_state = BoardState()
        # 18) Explicit casting to satisfy ROS 2 strict type checks
        new_state.cells = [int(x) for x in working_cells]
        new_state.last_move_cell = int(final_last_move_cell)
        new_state.last_move_player = int(final_last_move_player)
        
        response.state = new_state
        return response

def main(args=None):
    rclpy.init(args=args)
    ui_node = UserInterfaceNode()
    
    try:
        # 3) Single Thread Spin
        rclpy.spin(ui_node)
    except KeyboardInterrupt:
        pass
    finally:
        ui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
