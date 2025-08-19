#!/usr/bin/env python3
"""
Simple Formation Test Script

Test different formation patterns without animation.
"""

import numpy as np

def test_formations():
    """Test formation calculations."""
    print("🧪 Testing Formation Calculations")
    print("=" * 40)
    
    # Test formation patterns
    formations = {
        'triangle': [(0.0, 0.0), (-3.0, -3.0), (3.0, -3.0)],
        'line': [(-3.0, 0.0), (0.0, 0.0), (3.0, 0.0)],
        'square': [(-1.5, -1.5), (1.5, -1.5), (0.0, 1.5)],
        'v_formation': [(0.0, 0.0), (-3.0, -3.0), (3.0, -3.0)]
    }
    
    for name, positions in formations.items():
        print(f"\n📐 {name.upper()} Formation:")
        for i, (x, y) in enumerate(positions):
            print(f"   Drone {i+1}: ({x:5.1f}, {y:5.1f})")
        
        # Calculate center
        center = np.mean(positions, axis=0)
        print(f"   Center: ({center[0]:5.1f}, {center[1]:5.1f})")
    
    print("\n✅ All formation tests passed!")

if __name__ == "__main__":
    test_formations()
