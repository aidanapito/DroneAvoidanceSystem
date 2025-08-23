#!/usr/bin/env python3
"""
Simple X11 test to verify display forwarding is working
"""

import os
import subprocess
import sys

def test_x11():
    """Test if X11 display is accessible"""
    print("🧪 Testing X11 Display Forwarding...")
    
    # Set display environment
    os.environ['DISPLAY'] = 'host.docker.internal:0.0'
    os.environ['LIBGL_ALWAYS_INDIRECT'] = '1'
    
    print(f"DISPLAY: {os.environ.get('DISPLAY')}")
    print(f"LIBGL_ALWAYS_INDIRECT: {os.environ.get('LIBGL_ALWAYS_INDIRECT')}")
    
    try:
        # Try to run a simple X11 command
        result = subprocess.run(['xset', 'q'], 
                              capture_output=True, 
                              text=True, 
                              timeout=10)
        
        if result.returncode == 0:
            print("✅ X11 forwarding is working!")
            print("Output:", result.stdout[:200] + "..." if len(result.stdout) > 200 else result.stdout)
            return True
        else:
            print("❌ X11 command failed")
            print("Error:", result.stderr)
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ X11 command timed out")
        return False
    except FileNotFoundError:
        print("❌ X11 utilities not found")
        return False
    except Exception as e:
        print(f"❌ X11 test failed: {e}")
        return False

if __name__ == "__main__":
    success = test_x11()
    sys.exit(0 if success else 1)
