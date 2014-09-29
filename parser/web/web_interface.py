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
# Globals
########################################################################

app = Flask(__name__)
frontend = None


@app.route('/style.css')
def style():
    return app.send_static_file('style.css')


@app.route('/', methods=['GET', 'POST'])
def parse():
    try:
        objs_str = frontend.get_world_objects_str()
        robot_str = frontend.get_robot_str()

        if request.method == 'POST':
            if request.form['type'] == 'describe':
                res = frontend.describe()
                debug = frontend.get_buffer()
            elif request.form['type'] == 'ground':
                data = request.form['groundquery']
                res = frontend.ground(data)
                debug = frontend.get_buffer()
            else:
                # request.form['type'] == 'parse'
                data = request.form['inputtext']
                res = frontend.parse(data)
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


def main(args=[]):
    # Check args
    useros = len(args) == 0 or args[0] != 'noros'

    # Make parser frontend, enabling ROS if desired.
    global frontend
    frontend = WebFrontend()
    if useros:
        frontend.startup_ros(spin=False)
    else:
        frontend.set_default_world()

    # Serve
    app.run(debug=True, use_reloader=False)


if __name__ == '__main__':
    main(sys.argv[1:])
