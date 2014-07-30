'''Web interface for hfpbd-parser.'''

__author__ = 'mbforbes'


########################################################################
# Imports
########################################################################

# Builtins
import os
import traceback

# 3rd party
from flask import Flask, render_template, request

# Local
from hybridbayes import Parser

########################################################################
# Main (execution starts here)
########################################################################

app = Flask(__name__)
parser = Parser()
parser.set_world()

@app.route('/style.css')
def style():
    return app.send_static_file('style.css')

@app.route('/', methods=['GET', 'POST'])
def parse():
    try:
        if request.method == 'POST':
            # TODO process
            data = request.form['inputtext']
            res = parser.parse(data)
            return render_template(
                'template.html',
                response=res,
                debug=''
            )
        else:
            # GET: just show form
            return render_template('template.html')
    except:
        print traceback.format_exc()

if __name__ == '__main__':
    app.run(debug=True)
