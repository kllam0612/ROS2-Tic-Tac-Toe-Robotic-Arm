#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Node: game_logic_node
Description: Stateless Service Server for Tic-Tac-Toe Logic.
Architecture: 
    - Input: Standard Row-Major Array (Index 0 = Top-Left)
    - Logic: Minimax Algorithm with Alpha-Beta Pruning
    - Output: Physical Keypad Mapping (Cell 9 = Top-Left)
"""
import yaml
import sys
from ament_index_python.packages import get_package_share_directory
import os
import random
import rclpy
from rclpy.node import Node

# Import Custom Interfaces
from ttt_interfaces.srv import GetGameStatus, GetRobotMove

class GameLogicNode(Node):
    def __init__(self):
        super().__init__('game_logic_node')

        # --- YAML LOADING ---
        config_file = os.path.join(
            get_package_share_directory('ttt_main'),
            'config',
            'game_logic_config.yaml'
        )
        
        try:
            with open(config_file, 'r') as file:
                data = yaml.safe_load(file)
                # Access the nested keys
                game_params = data['game_params']['ros__parameters']['game_logic_settings']
                
                # Assign to internal variable
                self.max_depth = int(game_params['depth'])

                self.get_logger().info(f"YAML Loaded Difficulty Depth: {self.max_depth}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML: {e}")
            # Fallback defaults
            self.difficulty_depth = 1

        self.declare_parameter('robot_symbol', 1)
        self.declare_parameter('human_symbol', 2)
        self.declare_parameter('empty_symbol', 0)

        self.robot_symbol = self.get_parameter('robot_symbol').value
        self.human_symbol = self.get_parameter('human_symbol').value
        self.empty_symbol = self.get_parameter('empty_symbol').value

        # ---- Services ----
        self.srv_status = self.create_service(
            GetGameStatus,
            'get_game_status',
            self.handle_get_game_status
        )

        self.srv_move = self.create_service(
            GetRobotMove,
            'get_robot_move',
            self.handle_get_robot_move
        )

        # ---- Constants ----
        # Win Lines (Indices 0-8)
        self.WIN_LINES = [
            (0, 1, 2), (3, 4, 5), (6, 7, 8),  # Rows
            (0, 3, 6), (1, 4, 7), (2, 5, 8),  # Cols
            (0, 4, 8), (2, 4, 6)              # Diagonals
        ]

        self.get_logger().info("Game Logic Node Ready.")

    # ---------------------------------------------------------------------
    # Service Callbacks
    # ---------------------------------------------------------------------

    def handle_get_game_status(self, request, response):
        """
        Determines the current game status.
        Request: BoardState state (containing int32[] cells)
        Response: int32 status (0=Ongoing, 1=RobotWin, 2=HumanWin, 3=Draw)
        """
        try:
            board = list(request.state.cells)
            response.status = int(self._check_winner(board))
            self.get_logger().info("GetGameStatus called")
            self.get_logger().info(f"Status Checked: {response.status}")
        except Exception as e:
            self.get_logger().error(f"Status Check Failed: {e}")
            response.status = -1 # Error
        
        return response

    def handle_get_robot_move(self, request, response):
        """
        Calculates the best move for the robot.
        Request: BoardState state
        Response: int32 target_cell (1-9 Keypad Layout)
        """
        try:
            board = list(request.state.cells)
            self.get_logger().info("GetRobotMove called")
        except Exception as e:
            self.get_logger().error(f"Failed to parse board: {e}")
            response.target_cell = -1
            return response
        
        # 1. First Turn / Early Game Optimization
        # If board is empty, pick random cell
        if all(x == self.empty_symbol for x in board):
            target_cell = random.randint(1, 9)
            self.get_logger().info(f"Empty Board: Random Move to Cell {target_cell}")
            response.target_cell = int(target_cell)
            return response

        # 2. Safety Check: Is board full or already won?
        current_status = self._check_winner(board)
        if current_status != 0:
            self.get_logger().warn("Move requested but game is already over.")
            response.target_cell = -1
            return response

        # 3. Mid-Game Logic: Minimax with Alpha-Beta Pruning
        self.get_logger().info("Calculating Best Move...")
        best_score = -float('inf')
        best_move_index = -1
        
        # Alpha-Beta Initial Values
        alpha = -float('inf')
        beta = float('inf')

        for i in range(9):
            if board[i] == self.empty_symbol:
                # Tentative Move
                board[i] = self.robot_symbol
                
                # Call Minimax (Next turn is Minimizer/Human)
                score = self._minimax(board, 0, False, alpha, beta)
                
                # Undo Move
                board[i] = self.empty_symbol

                if score > best_score:
                    best_score = score
                    best_move_index = i
                
                # Update Alpha for the root level
                alpha = max(alpha, best_score)

        # 4. Generate Output
        if best_move_index != -1:
            # Map Internal Index (0-8) to User Cell (9-1)
            target_cell = 9 - best_move_index
            self.get_logger().info(f"Best Move: Index {best_move_index} -> Cell {target_cell} (Score: {best_score})")
            response.target_cell = int(target_cell)
        else:
            self.get_logger().error("Logic Error: No valid moves found on ongoing board.")
            response.target_cell = -1

        return response

    # ---------------------------------------------------------------------
    # Core Logic Methods
    # ---------------------------------------------------------------------

    def _check_winner(self, board):
        """
        Returns: 0=Ongoing, 1=Robot, 2=Human, 3=Draw
        """
        # Check Lines
        for a, b, c in self.WIN_LINES:
            if board[a] == board[b] == board[c] and board[a] != self.empty_symbol:
                return board[a] 
        # Check Draw (Board Full)
        if self.empty_symbol not in board:
            return 3
        return 0

    def _minimax(self, board, depth, is_maximizing, alpha, beta):
        # Check Terminal States (Win/Loss)
        status = self._check_winner(board)
        if status == self.robot_symbol:
            return 10 - depth
        elif status == self.human_symbol:
            return -10 + depth
        elif status == 3:
            return 0
        
        # Depth Limit Check
        if depth >= self.max_depth:
            return 0

        if is_maximizing:
            max_eval = -float('inf')
            for i in range(9):
                if board[i] == self.empty_symbol:
                    board[i] = self.robot_symbol
                    eval_score = self._minimax(board, depth + 1, False, alpha, beta)
                    board[i] = self.empty_symbol
                    
                    max_eval = max(max_eval, eval_score)
                    alpha = max(alpha, eval_score)
                    if beta <= alpha:
                        break # Beta Cut-off
            return max_eval
        else:
            min_eval = float('inf')
            for i in range(9):
                if board[i] == self.empty_symbol:
                    board[i] = self.human_symbol
                    eval_score = self._minimax(board, depth + 1, True, alpha, beta)
                    board[i] = self.empty_symbol
                    
                    min_eval = min(min_eval, eval_score)
                    beta = min(beta, eval_score)
                    if beta <= alpha:
                        break # Alpha Cut-off
            return min_eval

def main(args=None):
    rclpy.init(args=args)
    node = GameLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
