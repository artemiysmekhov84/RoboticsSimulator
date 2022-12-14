import datetime
import numpy
import cv2
import pybullet
import pybullet_data
import flask
import waitress

# Setup parameters.
frameSize = (640, 480)  # Resolution of the syntheric camera.
useRealTimeSimulation = False  # Indicates to call `stepSimulation()` method automatically as fast as possible.

renderingTimeSeconds = 0.0
renderedFramesNumber = 0
fps = 0.0

# Initialize pybullet environment.
pybullet.connect(pybullet.DIRECT)  # GUI/DIRECT
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
pybullet.setGravity(0.0, 0.0, -10.0)
pybullet.setRealTimeSimulation(useRealTimeSimulation)
pybullet.loadURDF("plane.urdf")

# TODO: Do I need this debug visualizer configuration? Seems it has no effect on performance.
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

# Create robot and camera position matrix.
robotPosition = (0.0, 0.0, 1.0)
robotOrientation = pybullet.getQuaternionFromEuler((0.0, 0.0, 0.0))
robot = pybullet.loadURDF("r2d2.urdf", robotPosition, robotOrientation)
viewMatrix = pybullet.computeViewMatrix(
    cameraEyePosition=(2.0, 2.0, 2.0), cameraTargetPosition=(0.0, 0.0, 0.0), cameraUpVector=(0.0, 0.0, 1.0)
)

# Create Flask app.
app = flask.Flask(__name__)
# app.secret_key = "131200e9e5bc8f3f2a5b88d844f5ea45820da7cb0e026b5b56bc2e61dc7cfb8b"  # TODO: Do I need this?


@app.route("/stream", methods=["GET"])
def stream():
    return flask.Response(streamingLoop(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/reset", methods=["POST"])
def reset():
    pybullet.resetBasePositionAndOrientation(robot, robotPosition, robotOrientation)
    return flask.Response()


def captureFrame():
    global renderingTimeSeconds, renderedFramesNumber, fps

    startTime = datetime.datetime.now()

    image = numpy.zeros((frameSize[0] * frameSize[1] * 4,))  # Empty debug image.
    _, _, image, _, _ = pybullet.getCameraImage(
        width=frameSize[0],
        height=frameSize[1],
        viewMatrix=viewMatrix,
        shadow=False,
        renderer=pybullet.ER_TINY_RENDERER,  # ER_TINY_RENDERER/ER_BULLET_HARDWARE_OPENGL
    )
    frame = numpy.reshape(image, (frameSize[1], frameSize[0], 4))[:, :, 2::-1]  # Invert channels order.
    frame = cv2.normalize(frame, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    information = f"FPS: {fps:.2f} / time: {datetime.datetime.now()}"
    frame = cv2.putText(frame, information, (5, 15), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255))

    endTime = datetime.datetime.now()
    renderingTimeSeconds += (endTime - startTime).total_seconds()
    renderedFramesNumber += 1
    fps = renderedFramesNumber / renderingTimeSeconds

    return frame


def streamingLoop():
    while True:
        if not useRealTimeSimulation:
            pybullet.stepSimulation()

        frame = captureFrame()

        # Encode the frame in JPEG format.
        isEncoded, encodedFrame = cv2.imencode(".jpg", frame)
        if not isEncoded:
            continue

        # Yield the output frame in the byte format.
        yield (b"--frame\r\n" b"Content-Type: image/jpg\r\n\r\n" + bytearray(encodedFrame) + b"\r\n")


# Start server listening at localhost:8080.
waitress.serve(app)
