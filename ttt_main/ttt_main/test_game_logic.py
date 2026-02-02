#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Node: test_game_logic_manual
Description: Client node to send USER-DEFINED requests to game_logic_node 
             and verify Minimax/Status logic.
"""

import rclpy
from rclpy.node import Node
import time
import sys

# Imports from your custom interface package
from ttt_interfaces.srv import GetGameStatus, GetRobotMove
from ttt_interfaces.msg import BoardState

class GameLogicTester(Node):
    def __init__(self):
        super().__init__('test_game_logic')
        
        # Create Clients
        self.cli_status = self.create_client(GetGameStatus, 'get_game_status')
        self.cli_move = self.create_client(GetRobotMove, 'get_robot_move')

        # Wait for services
        self.get_logger().info('Waiting for game_logic_node services...')
        while not self.cli_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_game_status not available, waiting...')
        while not self.cli_move.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service get_robot_move not available, waiting...')
            
        self.get_logger().info('Services Found! Ready for Manual Input.\n')

    def run_manual_test(self, board_array):
        """
        Sends requests based on user input and prints the result.
        """
        self.get_logger().info(f"Sending Board: {board_array}")
        
        # 1. Create Request Object
        req_status = GetGameStatus.Request()
        req_status.state = BoardState()
        req_status.state.cells = board_array
        
        req_move = GetRobotMove.Request()
        req_move.state = BoardState()
        req_move.state.cells = board_array

        # 2. Call Services
        future_status = self.cli_status.call_async(req_status)
        rclpy.spin_until_future_complete(self, future_status)
        res_status = future_status.result()

        future_move = self.cli_move.call_async(req_move)
        rclpy.spin_until_future_complete(self, future_move)
        res_move = future_move.result()

        # 3. Validation & Printing
        status_map = {0: "Ongoing", 1: "Robot Win", 2: "Human Win", 3: "Draw"}
        status_str = status_map.get(res_status.status, "Unknown")
        
        print("\n" + "="*30)
        print(f"REPORT for Input: {board_array}")
        print(f"   -> Game Status: {res_status.status} ({status_str})")
        print(f"   -> Robot Move:  Cell {res_move.target_cell}")
        print("="*30 + "\n")

def get_user_input():
    """
    Prompts user for board state.
    Returns: List[int] of length 9, or None if exit.
    """
    print("\n--- Manual Board Input ---")
    print("Enter 9 numbers separated by space (Row-Major: 0=Empty, 1=Robot, 2=Human)")
    print("Format: [Top-L Top-C Top-R  Mid-L Mid-C Mid-R  Bot-L Bot-C Bot-R]")
    print("Example: 1 1 0 0 2 0 0 0 0")
    print("Type 'q' or 'exit' to quit.")
    
    user_str = input("Board > ")
    
    if user_str.lower() in ['q', 'exit']:
        return None
    
    try:
        # Parse string into list of integers
        items = user_str.strip().split()
        if len(items) != 9:
            print(f"[ERROR] Expected 9 numbers, got {len(items)}.")
            return get_user_input() # Recursive retry
            
        board = [int(x) for x in items]
        
        # Validate values (optional but good)
        if any(x not in [0, 1, 2] for x in board):
             print("[WARN] Detected values other than 0, 1, 2. Sending anyway...")
             
        return board
    except ValueError:
        print("[ERROR] Invalid input. Please enter integers only.")
        return get_user_input()

def main(args=None):
    rclpy.init(args=args)
    tester = GameLogicTester()

    try:
        while True:
            board = get_user_input()
            if board is None:
                print("Exiting...")
                break
            
            tester.run_manual_test(board)
            
    except KeyboardInterrupt:
        print("\nForce Quit.")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
