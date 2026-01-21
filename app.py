#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from flask import Flask, render_template, jsonify
import socket

app = Flask(__name__)

# Configuration
ROBOT_HOST = 'localhost'
ROBOT_PORT = 5055
SOCKET_TIMEOUT = 2  # seconds

def send_command_to_robot(command):
    """Send a command to the robot via socket connection."""
    try:
        # Create a socket connection
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client.settimeout(SOCKET_TIMEOUT)
        
        # Connect to robot
        client.connect((ROBOT_HOST, ROBOT_PORT))
        
        # Send command
        client.send(command.encode('utf-8'))
        
        # Close connection
        client.close()
        
        return True, f"Command '{command}' sent successfully"
    except Exception as e:
        return False, f"Error sending command: {str(e)}"

@app.route('/')
def index():
    """Render the control panel."""
    return render_template('index.html')

@app.route('/cmd/<command>')
def send_command(command):
    """Send a command to the robot."""
    success, message = send_command_to_robot(command)
    
    return jsonify({
        'success': success,
        'message': message,
        'command': command
    })

if __name__ == '__main__':
    print("=" * 50)
    print("SpotMicro Web Control Panel")
    print("=" * 50)
    print(f"Connecting to robot at {ROBOT_HOST}:{ROBOT_PORT}")
    print("Starting web server at http://localhost:5000")
    print("=" * 50)
    app.run(host='0.0.0.0', port=5000, debug=True)
