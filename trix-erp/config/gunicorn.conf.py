# gunicorn.conf.py
import os

bind = f"0.0.0.0:{os.environ.get('PORT', 6000)}"
loglevel = 'debug'
workers = 1
