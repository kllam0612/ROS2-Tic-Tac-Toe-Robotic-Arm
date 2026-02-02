#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from ttt_interfaces.srv import GetHumanMove, ManualUpdate
from ttt_interfaces.msg import BoardState

class UITestClient(Node):
    def __init__(self):
        super().__init__('ui_test_client')
        
        self.cli_get_move = self.create_client(GetHumanMove, 'get_human_move')
        self.cli_manual = self.create_client(ManualUpdate, 'manual_update')

        self.get_logger().info('Waiting for UI Node services...')
        if not self.cli_get_move.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service "get_human_move" not available.')
        if not self.cli_manual.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service "manual_update" not available.')
            
        self.get_logger().info('Services found! Starting interactive tester...')

    def generate_board_state(self):
        """
        Helper to create a custom board state for testing.
        Now includes prompts for last_move_cell and last_move_player.
        """
        print("\n--- Configure Test Board State ---")
        print("Enter 9 integers for cells (0=Empty, 1=Robot, 2=Human).")
        print("Example: 0 0 0 0 1 0 0 0 2")
        
        state = BoardState()

        # 1. Custom Cells
        try:
            raw = input("Cells [Default: All 0]: ")
            if not raw.strip():
                state.cells = [0] * 9
            else:
                cells = [int(x) for x in raw.split()]
                if len(cells) != 9:
                    raise ValueError("Must provide exactly 9 numbers.")
                state.cells = cells
        except ValueError as e:
            print(f"Invalid input: {e}. Using empty board.")
            state.cells = [0] * 9

        # 2. Custom Last Move Cell (The missing criteria)
        try:
            raw_cell = input("Last Move Cell (1-9) [Default: -1]: ")
            state.last_move_cell = int(raw_cell) if raw_cell.strip() else -1
        except ValueError:
            print("Invalid input. Using 0.")
            state.last_move_cell = 0

        # 3. Custom Last Move Player (The missing criteria)
        try:
            raw_player = input("Last Move Player (-1=None, 1=Bot, 2=Human) [Default: -1]: ")
            state.last_move_player = int(raw_player) if raw_player.strip() else -1
        except ValueError:
            print("Invalid input. Using 0.")
            state.last_move_player = 0

        return state

    def call_get_human_move(self):
        req = GetHumanMove.Request()
        req.state = self.generate_board_state()
        
        print("\n[Sending Request] Waiting for Human Input on UI Node...")
        future = self.cli_get_move.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            print(f"\n[Response Received]")
            print(f"Selected Cell: {response.target_cell}")
        except Exception as e:
            print(f"Service call failed: {e}")
        print("--------------------------------------------------")

    def call_manual_update(self):
        req = ManualUpdate.Request()
        req.state = self.generate_board_state()
        
        print("\n[Sending Request] Waiting for Manual Update on UI Node...")
        future = self.cli_manual.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            res = future.result()
            print(f"\n[Response Received]")
            print(f"New Board: {res.state.cells}")
            print(f"Last Move Cell: {res.state.last_move_cell}")
            print(f"Last Move Player: {res.state.last_move_player}")
        except Exception as e:
            print(f"Service call failed: {e}")
        print("--------------------------------------------------")

def main():
    rclpy.init()
    client = UITestClient()

    try:
        while rclpy.ok():
            print("\n=== UI TEST CLIENT MENU ===")
            print("1. Test 'GetHumanMove'")
            print("2. Test 'ManualUpdate'")
            print("q. Quit")
            choice = input("Select Option: ")

            if choice == '1':
                client.call_get_human_move()
            elif choice == '2':
                client.call_manual_update()
            elif choice.lower() == 'q':
                break
            else:
                print("Invalid choice.")
                
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
