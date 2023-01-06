FROM robotlocomotion/drake:1.11.0

ADD main.py .

CMD [ "python", "./main.py" ]
