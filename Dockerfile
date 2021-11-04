FROM python:3.9-slim-buster

# Create default user folder
RUN mkdir -p /home/pi

COPY cerulean /home/pi/cerulean
RUN apt-get update && apt-get install -y git

RUN cd /home/pi/dvl-a50 && pip3 install .  
RUN pip3 install pyserial git+https://github.com/CeruleanSonar/pynmea2

#ENTRYPOINT /home/pi/dvl-a50/main.py
ENTRYPOINT /home/pi/cerulean/main.py
