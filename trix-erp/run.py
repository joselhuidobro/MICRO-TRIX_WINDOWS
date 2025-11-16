# run.py
from app import create_app
import os
import psycopg2
import logging
from flask import Flask, render_template, request, redirect, url_for, flash
from flask_login import LoginManager, UserMixin, login_user, login_required, logout_user, current_user


app = create_app()


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 6800))
    app.run(debug=False, host='0.0.0.0', port=port)
