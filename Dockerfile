FROM python:3.9-slim-buster

# Create default user folder
RUN mkdir -p /home/pi/cerulean/
RUN apt-get update && apt-get install -y git
RUN pip3 install pyserial git+https://github.com/CeruleanSonar/pynmea2

COPY cerulean/setup.py /home/pi/cerulean/setup.py
RUN cd /home/pi/cerulean && pip3 install .

RUN apt-get install -y nano iputils-ping
COPY cerulean/src /home/pi/cerulean/src

#ENTRYPOINT /home/pi/dvl-a50/main.py
ENTRYPOINT /home/pi/cerulean/src/main.py
