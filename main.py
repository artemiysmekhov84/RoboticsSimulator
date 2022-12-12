import threading
import numpy
import cv2
import pybullet
import pybullet_data
from flask import Flask
from flask.wrappers import Response

# Setup parameters.
host = "192.168.0.102"  # IP-address of the streaming server.
port = 8000  # Port number of the streaming server.
frameSize = (256, 256)  # Resolution of the syntheric camera.
useRealTimeSimulation = True  # Indicates to call `stepSimulation()` method automatically as fast as possible.

app = Flask(__name__)

# Initialize a lock used to ensure thread-safe exchanges of the frames. It's required to support multiple clients.
lock = threading.Lock()

# Initialize pybullet environment.
pybullet.connect(pybullet.GUI)  # GUI/DIRECT
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0.0, 0.0, -10.0)
pybullet.setRealTimeSimulation(useRealTimeSimulation)
pybullet.loadURDF("plane.urdf")
pybullet.loadURDF("r2d2.urdf", (0.0, 0.0, 1.0), pybullet.getQuaternionFromEuler((0.0, 0.0, 0.0)))
viewMatrix = pybullet.computeViewMatrix(
    cameraEyePosition=(2.0, 2.0, 2.0), cameraTargetPosition=(0.0, 0.0, 0.0), cameraUpVector=(0.0, 0.0, 1.0)
)


@app.route("/stream", methods=["GET"])
def stream():
    return Response(getFrame(), mimetype="multipart/x-mixed-replace; boundary=frame")


def getFrame():
    global lock

    while True:
        with lock:  # Wait until the lock is acquired.
            if not useRealTimeSimulation:
                pybullet.stepSimulation()
            images = pybullet.getCameraImage(frameSize[0], frameSize[1], viewMatrix)
            frame = numpy.reshape(images[2], (*frameSize, 4))[:, :, 2::-1]  # Invert channels order.
            isEncoded, encodedFrame = cv2.imencode(".jpg", frame)  # Encode the frame in JPEG format.
            if not isEncoded:
                continue

        # Yield the output frame in the byte format.
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + bytearray(encodedFrame) + b"\r\n")


if __name__ == "__main__":
    app.run(host, port)  # TODO: For debug only, do not use this method in production.
