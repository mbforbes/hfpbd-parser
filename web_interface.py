'''Web interface for hfpbd-parser.'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import os
import sys
import traceback
import yaml

# 3rd party
from flask import Flask, render_template, request

# Local
from hybridbayes import Parser

########################################################################
# Main (execution starts here)
########################################################################

app = Flask(__name__)

# Make parser stuff
parser = Parser(buffer_printing=True)
parser.set_default_world()

# Get vars to display
print 'loading'
world = yaml.load(open('world.yml'))
objs = world['objects']
robot = world['robot']
objs_str = yaml.dump(objs)
robot_str = yaml.dump(robot)


@app.route('/style.css')
def style():
    return app.send_static_file('style.css')


@app.route('/', methods=['GET', 'POST'])
def parse():
    try:
        if request.method == 'POST':
            # TODO process
            data = request.form['inputtext']
            res, debug = parser.parse(data)
            return render_template(
                'template.html',
                response=str(res),
                debug=debug,
                objs=objs_str,
                robot=robot_str
            )
        else:
            # GET: just show form
            return render_template(
                'template.html',
                objs=objs_str,
                robot=robot_str
            )
    except:
        print traceback.format_exc()

if __name__ == '__main__':
    print sys.argv
    app.run(debug=True)
