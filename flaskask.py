# requires python2.7
from flask import Flask, render_template  # pylint: disable=E0401
from flask_ask import Ask, statement, question, session  # pylint: disable=E0401
import json
import requests
import time
import sys
from jinja2 import Template
import subprocess

app = Flask(__name__, template_folder='template')
ask = Ask(app, '/')

@ask.launch
def launch():
    subprocess.call(['./scripts/pythontest.sh'])
    return statement('launch')

@ask.intent('HelloIntent')
def hello():
    return statement('hi. hello intent worked.')

@ask.intent('NodIntent')
def nod():
    subprocess.call(['./launch_nod.sh'])    
    return statement('rover, are you listening?')

@ask.intent('ShakeIntent')
def shake():
    subprocess.call(['./launch_shake.sh'])
    return statement(['rover. shake.'])


if __name__ == '__main__':
    app.run(debug=True,host='0.0.0.0', port=8000)