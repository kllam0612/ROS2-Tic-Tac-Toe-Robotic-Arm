#!/usr/bin/env python3
import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

# 14) Custom service; all services import from ttt_interfaces
from ttt_interfaces.msg import BoardState
from ttt_interfaces.srv import (
    GetBoardState, 
    GetHumanMove, 
    ManualUpdate, 
    GetRobotMove, 
    GetGameStatus
)
from ttt_interfaces.action import ExecuteMove, TextToSpeech

class MainOrchestrator(Node): # 1) class MainOrchestrator
    def __init__(self):
        # 1) initialize as main_orchestrator
        super().__init__('main_orchestrator')

        # 2) Internal memory
        self.state = BoardState()
        self.state.cells = [0] * 9  # Initialize empty
        self.state.last_move_cell = 0
        self.state.last_move_player = 0
        self.target_cell = 0 # 1-9

        # 4) Call services & actions (Clients)
        self.cli_get_board = self.create_client(GetBoardState, 'get_board_state')
        self.cli_get_human = self.create_client(GetHumanMove, 'get_human_move')
        self.cli_manual = self.create_client(ManualUpdate, 'manual_update')
        self.cli_get_robot = self.create_client(GetRobotMove, 'get_robot_move')
        self.cli_game_status = self.create_client(GetGameStatus, 'get_game_status')

        self.act_execute_move = ActionClient(self, ExecuteMove, 'execute_move')
        self.act_tts = ActionClient(self, TextToSpeech, 'text_to_speech')
        
        self.get_logger().info("Main Orchestrator Ready.")

        # Wait for interfaces (Basic check to ensure system is up)
        self.cli_get_board.wait_for_service(timeout_sec=5.0)
        self.cli_get_human.wait_for_service(timeout_sec=5.0)
        self.cli_manual.wait_for_service(timeout_sec=5.0)
        self.cli_get_robot.wait_for_service(timeout_sec=5.0)
        self.cli_game_status.wait_for_service(timeout_sec=5.0)
        # Actions are often slower to discover; we wait in the flow if needed or here
        self.act_execute_move.wait_for_server(timeout_sec=10.0)
        self.act_tts.wait_for_server(timeout_sec=10.0)
        
        #self.get_logger().info("Interfaces ready.")

    # --- Helper: Cell Mapping (13, 15, 16) ---
    def index_to_cell(self, index):
        """Maps index 0..8 to cell 9..1"""
        # Layout: 987 / 654 / 321
        # Index:  012 / 345 / 678
        return 9 - index

    def cell_to_index(self, cell):
        return 9 - cell

    # --- Helper: Display ASCII Board ---
    def display_ascii(self):
        """Displays BoardState.msg including last_move details."""
        cells = self.state.cells
        rows = []
        for r in range(3):
            line = ""
            for c in range(3):
                idx = r * 3 + c
                cell_num = self.index_to_cell(idx)
                val = cells[idx]
                char = '.'
                if val == 1: char = 'R'
                elif val == 2: char = 'H'
                line += f"{cell_num}[{char}] "
            rows.append(line)
        
        self.get_logger().info("\n" + "\n".join(rows))
        self.get_logger().info(f"Last Move Cell: {self.state.last_move_cell}, Player: {self.state.last_move_player}")

    # --- Helper: Synchronous Wrappers for Single Thread (3) ---
    def call_service(self, client, request):
        """Helper to call a service and block until response received."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def call_action(self, client, goal_msg):
        """Helper to call an action and block until result received."""
        # Send Goal
        send_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Action goal rejected")
            return None, None

        # Get Result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result_wrapper = res_future.result()
        
        # We also need the feedback. 
        # Note: In strict sync wrapper, capturing feedback continuously is tricky without a callback.
        # This wrapper returns the final result. Feedback logging is handled via callback in main flow if needed.
        return result_wrapper.result, None # logic handled inside main for feedback

    # --- Main Logic Sequence (17) ---
    def run_sequence(self):
        # [Setup]
        self.state = BoardState()
        self.target_cell = 0

        # [Reset]
        req = GetBoardState.Request()
        req.reset_diff_tracker = True
        resp = self.call_service(self.cli_get_board, req)
        self.state = resp.state
        self.display_ascii()
        """
        # [Manual]
        while True:
            ans = input("[Manual] Perform manual update? (y/n): ").strip().lower()
            if ans == 'y':
                req_man = ManualUpdate.Request()
                req_man.state = self.state
                resp_man = self.call_service(self.cli_manual, req_man)
                self.state = resp_man.state
                break
            elif ans == 'n':
                break
            else:
                print("Invalid input. Please type 'y' or 'n'.")
        """
        # [Sound]
        self._play_sound("Hello! I am Ros Master Robot. Welcome to play tic-tac-toe with me. You will lose. This is going to be fun!")
        time.sleep(2.0)

        # [Loop Begin]
        self.get_logger().info("Enter Main Loop")
        
        while rclpy.ok():
            # [Win Check]
            status = self._check_win_status()
            if status != 0: break # 0=Ongoing, 1=Robot, 2=Human, 3=Draw

            # [Sound]
            self._play_sound("Now is my turn. Where should I move? Let me think...")
            time.sleep(2.0)
            self._play_sound("Well, should I move to here?")
            time.sleep(2.0)
            
            # [Robot Turn]
            req_rob = GetRobotMove.Request()
            req_rob.state = self.state
            resp_rob = self.call_service(self.cli_get_robot, req_rob)
            self.target_cell = resp_rob.target_cell
            self.get_logger().info(f"[Robot Turn] Target Cell: {self.target_cell}")

            # [Sound]
            self._play_sound(f"Alright. My move is {self.target_cell}. This is going to be exciting!")
            time.sleep(2.0)
            """
            # [Supply]
            while True:
                ans = input("[Supply] Piece ready at supply? (y): ").strip().lower()
                if ans == 'y': break
                else:
                   print("Invalid input. Please type 'y' only.")
            """
            # [PnP]
            self._execute_pnp(self.target_cell)
            time.sleep(4.0)

            # [Observe]
            self._observe_board()

            # [Verify]
            self._verify_move()

            # [Manual]
            self._manual_prompt()

            # [Win Check]
            status = self._check_win_status()
            if status != 0: break

            # [Sound]
            self._play_sound("Now is your turn. Where will you move? Let me guess.")
            time.sleep(2.0)
            self._play_sound("You might move to here.")
            time.sleep(2.0)

            # [Human Turn]
            req_hum = GetHumanMove.Request()
            req_hum.state = self.state
            resp_hum = self.call_service(self.cli_get_human, req_hum)
            self.target_cell = resp_hum.target_cell
            self.get_logger().info(f"[Human Turn] Target Cell: {self.target_cell}")

            # [Sound]
            self._play_sound(f"Bravo. Your move is {self.target_cell}. Haha, I have foresee this!")
            time.sleep(2.0)
            """
            # [Supply]
            while True:
                ans = input("[Supply] Piece ready at supply? (y): ").strip().lower()
                if ans == 'y': break
                else:
                   print("Invalid input. Please type 'y' only.")
            """
            # [PnP]
            self._execute_pnp(self.target_cell)
            time.sleep(4.0)

            # [Observe]
            self._observe_board()

            # [Verify]
            self._verify_move()

            # [Manual]
            self._manual_prompt()

            # [Loop Continue] - Back to top

        # [Loop Exit]
        self.get_logger().info("Exit Main Loop")
        
        # Status Messages
        if status == 1:
            self._play_sound("Yay, I won. A.I. is unbeatable!")
            time.sleep(1.0)
        elif status == 2:
            self._play_sound("You won! How is that possible? I lost to you.")
            time.sleep(1.0)
        elif status == 3:
            self._play_sound("Oh my. This is a draw. Next time I will win you!")
            time.sleep(1.0)
 
        time.sleep(1.0)
        
        # [End]
        self._play_sound("Thank you for playing. Please fill in the survey form. See you again!")
        time.sleep(1.0)
        sys.exit()

    # --- Sub-routines to keep main loop clean ---

    def _play_sound(self, text):
        goal = TextToSpeech.Goal()
        goal.say = text
        # Send and wait
        send_future = self.act_tts.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        # Wait for result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        res = res_future.result().result
        
        # Display say_done only
        self.get_logger().info(f"[Sound Result] Say Done: {res.say_done}")

    def _execute_pnp(self, cell):
        """
        Executes Pick and Place action.
        Updated: Removes feedback callback (ignoring 'log1') and only returns final result.
        """
        goal = ExecuteMove.Goal()
        goal.target_cell = cell
        
        # Send goal WITHOUT feedback callback
        send_future = self.act_execute_move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("PnP Action Goal Rejected")
            return

        # Wait for result
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        # Log only the final result state
        self.get_logger().info(f"PnP Result: placed={result.placed}")

    def _observe_board(self):
        req = GetBoardState.Request()
        req.reset_diff_tracker = False
        resp = self.call_service(self.cli_get_board, req)
        self.state = resp.state
        self.display_ascii()

    def _verify_move(self):
        if self.state.last_move_cell == self.target_cell:
            # Added 'f' before the string
            self.get_logger().info(f"Move Executed Correctly: {self.target_cell}")
        else:
            self.get_logger().info(f"Error: Target was {self.target_cell}, but detected {self.state.last_move_cell}")

    def _manual_prompt(self):
        while True:
            ans = input("[Manual] Perform manual update? (y/n): ").strip().lower()
            if ans == 'y':
                req = ManualUpdate.Request()
                req.state = self.state
                self.state = self.call_service(self.cli_manual, req).state
                break
            elif ans == 'n':
                break
            else:
                print("Invalid input. Please type 'y' or 'n'.")

    def _check_win_status(self):
        req = GetGameStatus.Request()
        req.state = self.state
        resp = self.call_service(self.cli_game_status, req)
        self.get_logger().info(f"Game Status: {resp.status}")
        return resp.status # 0=Ongoing, 1=Robot, 2=Human, 3=Draw

def main(args=None):
    rclpy.init(args=args)
    node = MainOrchestrator()
    try:
        node.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
