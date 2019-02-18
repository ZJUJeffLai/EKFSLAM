FROM python:2.7

ENV PYTHONUNBUFFERED 1

WORKDIR /

RUN apt-get update && apt-get install -y -qq \
    python-numpy \
    python-matplotlib

RUN pip install numpy matplotlib