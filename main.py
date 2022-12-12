import datetime
import threading
import numpy
import cv2
import pybullet
import pybullet_data
from flask import Flask
from flask.wrappers import Response

# Setup parameters.
host = "192.168.0.102"  # IP-address of the streaming server.
port = 8080  # Port number of the streaming server.
frameSize = (256, 256)  # Resolution of the syntheric camera.
useRealTimeSimulation = False  # Indicates to call `stepSimulation()` method automatically as fast as possible.

app = Flask(__name__)
app.secret_key = "131200e9e5bc8f3f2a5b88d844f5ea45820da7cb0e026b5b56bc2e61dc7cfb8b"

# Initialize a lock used to ensure thread-safe exchanges of the frames. It's required to support multiple clients.
lock = threading.Lock()

# Initialize pybullet environment.
pybullet.connect(pybullet.DIRECT)  # GUI/DIRECT
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
            frame = cv2.normalize(frame, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

            timeString = str(datetime.datetime.now())
            frame = cv2.putText(frame, timeString, (5, 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))

            isEncoded, encodedFrame = cv2.imencode(".jpg", frame)  # Encode the frame in JPEG format.
            if not isEncoded:
                continue

        # Yield the output frame in the byte format.
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + bytearray(encodedFrame) + b"\r\n")


# waitress-serve --host='192.168.0.102' --call 'main:flaskApp'
def flaskApp():
    return app


if __name__ == "__main__":
    app.run(host, port)  # TODO: For debug only, do not use this method in production.
