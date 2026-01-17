import serial
import time
Import("env")

def after_upload(source, target, env):
    port = env.subst("$UPLOAD_PORT")
    print(f"Forcing reset on {port}...")
    try:
        ser = serial.Serial(port, 115200)
        ser.setDTR(False)
        ser.setRTS(True)
        time.sleep(0.1)
        ser.setRTS(False)
        ser.setDTR(True)
        time.sleep(0.1)
        ser.close()
        print("Reset complete!")
    except Exception as e:
        print(f"Reset failed: {e}")

env.AddPostAction("upload", after_upload)
