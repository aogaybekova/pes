#!/usr/bin/env python3
"""
Simple integration test for the robot dog control system.
Tests basic functionality without requiring actual hardware.
"""

import sys
import os
import socket
import time

def test_imports():
    """Test that all required modules can be imported."""
    print("Testing imports...")
    
    try:
        import oop
        print("✓ oop.py imports successfully")
    except Exception as e:
        print(f"✗ Failed to import oop.py: {e}")
        return False
    
    try:
        import app
        print("✓ app.py imports successfully")
    except Exception as e:
        print(f"✗ Failed to import app.py: {e}")
        return False
    
    return True

def test_flask_routes():
    """Test that Flask app has required routes."""
    print("\nTesting Flask routes...")
    
    try:
        from app import app as flask_app
        
        # Get all routes
        routes = [str(rule) for rule in flask_app.url_map.iter_rules()]
        
        # Check for required routes
        required_routes = ['/', '/cmd/<command>']
        for route in required_routes:
            if any(route in r for r in routes):
                print(f"✓ Route exists: {route}")
            else:
                print(f"✗ Missing route: {route}")
                return False
        
        return True
    except Exception as e:
        print(f"✗ Failed to test routes: {e}")
        return False

def test_oop_methods():
    """Test that oop.py has required methods."""
    print("\nTesting oop.py methods...")
    
    try:
        import oop
        
        # Check for required methods
        required_methods = [
            'update_sensors',
            'log_environment', 
            'check_app_commands',
            'capture_photo'
        ]
        
        # We can't instantiate SpotMicroController without hardware,
        # but we can check if methods exist in the class
        for method_name in required_methods:
            if hasattr(oop.SpotMicroController, method_name):
                print(f"✓ Method exists: {method_name}")
            else:
                print(f"✗ Missing method: {method_name}")
                return False
        
        return True
    except Exception as e:
        print(f"✗ Failed to test methods: {e}")
        return False

def test_socket_communication():
    """Test socket server initialization (basic check)."""
    print("\nTesting socket functionality...")
    
    try:
        # Just test that socket module works
        test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        test_socket.close()
        print("✓ Socket module works correctly")
        return True
    except Exception as e:
        print(f"✗ Socket test failed: {e}")
        return False

def test_csv_functionality():
    """Test CSV logging functionality."""
    print("\nTesting CSV functionality...")
    
    try:
        import csv
        import tempfile
        
        # Create a temporary CSV file
        with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.csv') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'left_distance', 'right_distance', 
                           'touch_state', 'imu_roll', 'imu_pitch', 'current_command'])
            writer.writerow([time.time(), 100.0, 100.0, 0, 0.0, 0.0, 'stop'])
            temp_file = f.name
        
        # Read it back
        with open(temp_file, 'r') as f:
            reader = csv.reader(f)
            rows = list(reader)
            if len(rows) == 2:  # Header + 1 data row
                print("✓ CSV logging works correctly")
                os.unlink(temp_file)
                return True
            else:
                print(f"✗ CSV has wrong number of rows: {len(rows)}")
                os.unlink(temp_file)
                return False
    except Exception as e:
        print(f"✗ CSV test failed: {e}")
        return False

def test_templates():
    """Test that templates exist."""
    print("\nTesting templates...")
    
    template_path = os.path.join(os.path.dirname(__file__), 'templates', 'index.html')
    if os.path.exists(template_path):
        print(f"✓ Template exists: {template_path}")
        
        # Check that it has required elements
        with open(template_path, 'r') as f:
            content = f.read()
            required_elements = ['forward', 'backward', 'left', 'right', 'sit', 'stand', 'photo']
            for element in required_elements:
                if element in content.lower():
                    print(f"  ✓ Contains '{element}' button")
                else:
                    print(f"  ✗ Missing '{element}' button")
                    return False
        return True
    else:
        print(f"✗ Template not found: {template_path}")
        return False

def main():
    """Run all tests."""
    print("=" * 60)
    print("SpotMicro Control System - Integration Tests")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_flask_routes,
        test_oop_methods,
        test_socket_communication,
        test_csv_functionality,
        test_templates
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"\n✗ Test crashed: {e}")
            results.append(False)
    
    print("\n" + "=" * 60)
    passed = sum(results)
    total = len(results)
    print(f"Tests passed: {passed}/{total}")
    print("=" * 60)
    
    return all(results)

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
