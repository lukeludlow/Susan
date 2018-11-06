from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
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
    #launch_msg = render_template('launch')
    #return statement(launch_msg)
    subprocess.call(['./scripts/pythontest.sh'])
    return statement('launch')

@ask.intent('HelloIntent')
def hello():
    #return statement('it worked')
    hi_message = render_template('hi. it worked.')
    return statement(hi_msg)


if __name__ == '__main__':
#    if sys.version_info.major < 3:
#        reload(sys)
#    sys.setdefaultencoding('utf-8')
    app.run(debug=True,host='0.0.0.0', port=8000)
