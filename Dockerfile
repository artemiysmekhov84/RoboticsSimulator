# Create Docker image based on Drake 1.11.0.
FROM robotlocomotion/drake:1.11.0

ADD main.py .
ADD double_pendulum.sdf .

CMD [ "python", "./main.py" ]
