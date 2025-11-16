FROM python:3.14-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    tcpdump iproute2 iputils-ping libpcap0.8 libpcap-dev ca-certificates curl \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
ENV PIP_NO_CACHE_DIR=1 PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONUNBUFFERED=1
RUN python -m pip install --upgrade pip wheel \
 && pip install -r requirements.txt

COPY agent.py .
CMD ["python", "-u", "agent.py"]

