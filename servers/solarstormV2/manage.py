from flask import Flask, render_template
from flask.ext.script import Manager

from modules.data_processing import retrieve_data, parse_data

app = Flask(__name__)
manager = Manager(app)


# Routes
@app.route('/')
def index():
    return render_template('../../servers/solarstormV2/templates/index.html')


# CLI commands
@manager.option('-d', '--dir', dest='destination', default=None,
                help='Data Retriever, use the right file path, to save the data to.')
def retrieve(destination):
    retrieve_data(destination)


@manager.option('-d', '--dir', dest='source', default=None,
                help='Data Parser, parse the data from a path and create a single csv file.')
def parse(source):
    parse_data(source)


if __name__ == '__main__':
    manager.run()
