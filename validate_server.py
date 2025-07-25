#!/usr/bin/env python3

"""
Simple Server Validation

This module validates that the model server is running and returns a valid response.

Author: Mi Yan
License: CC-BY-NC 4.0
Created: 2025-07-10
"""

import zmq
import numpy as np
import sys
import os


def validate_server(server_ip: str = "127.0.0.1", port: int = 8000, timeout: int = 5) -> bool:
    """
    Validate that the server is running and returns a valid dict.

    Args:
        server_ip: Server IP
        port: Server port
        timeout: Timeout in seconds

    Returns:
        True if server returns valid dict, False otherwise
    """
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.setsockopt(zmq.RCVTIMEO, timeout * 1000)

    try:
        socket.connect(f"tcp://{server_ip}:{port}")

        # Create test data matching agent.py format
        mock_image = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)
        mock_proprio = [np.random.randn(7) for _ in range(4)]

        test_data = {
            'front_view_image': [mock_image],
            'side_view_image': [mock_image],
            'proprio_array': mock_proprio,
            'text': 'Validation test instruction',
        }

        socket.send_pyobj(test_data)
        print(f"tcp://{server_ip}:{port}")
        response = socket.recv_pyobj()

        # Check if response is a valid dict
        if not isinstance(response, dict):
            print(f"✗ Server returned {type(response)}, expected dict")
            return False

        print(f"✓ Server at {server_ip}:{port} returned valid dict")
        return True
    except zmq.Again:
        print(f"✗ Server at {server_ip}:{port} timeout after {timeout}s")
        return False
    except Exception as e:
        print(f"✗ Error connecting to server at {server_ip}:{port}: {e}")
        return False


if __name__ == "__main__":
    server_ip = os.environ.get('SERVER_IP')
    port = int(os.environ.get('SERVER_PORT'))
    if not server_ip or not port:
        print("Please set SERVER_IP and SERVER_PORT environment variables.")
        sys.exit(1)
    validate_server(server_ip, port)