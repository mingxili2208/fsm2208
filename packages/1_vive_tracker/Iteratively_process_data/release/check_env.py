import sys
import pprint # 用来更美观地打印列表

print("--- Python Executable ---")
print(sys.executable)
print("\n--- Python Search Paths (sys.path) ---")
pprint.pprint(sys.path)

print("\n--- Attempting to import pycpd ---")
try:
    import pycpd
    print("SUCCESS: pycpd was imported successfully!")
    print(f"Found at: {pycpd.__file__}")
except ImportError:
    print("FAILURE: pycpd could not be imported.")