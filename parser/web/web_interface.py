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
from parser.core.frontends import WebFrontend

########################################################################
# Main (execution starts here)
########################################################################

app = Flask(__name__)

# Make parser frontend, enabling ROS if possible.
frontend = WebFrontend()
if not frontend.startup_ros(spin=False):
    frontend.set_default_world()


@app.route('/style.css')
def style():
    return app.send_static_file('style.css')


@app.route('/', methods=['GET', 'POST'])
def parse():
    try:
        objs_str = frontend.get_world_objects_str()
        robot_str = frontend.get_robot_str()

        if request.method == 'POST':
            # TODO process
            data = request.form['inputtext']
            res = frontend.parse(data)
            print res
            debug = frontend.get_buffer()
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
    app.run(debug=True)
