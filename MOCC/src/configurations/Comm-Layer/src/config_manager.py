import datetime
import os
import sqlite3
import json
import tango_db_helper as DbHelper
from flasgger import Swagger
from flask import Flask, jsonify, request
try:
    from urllib import quote  # Python 2.X
except ImportError:
    from urllib.parse import    quote  # Python 3+


if 'tango.db' not in os.listdir('.'):
    DbHelper.create()

app = Flask(__name__)
swagger = Swagger(app)


@app.errorhandler(500)
def internal_error(error):
    """500 Internal Server Handler
    ---
    tags:
      - 500
    """
    return "HTTP 500: Internal Server Error"


@app.route('/save/<path:tango_addr>/', methods=['GET'])
def save(tango_addr):
    """Endpoint to map a Tango device address to the sender's IP address
    ---
    tags:
      - save
    parameters:
      - name: tango_addr
        in: path
        type: string
        required: true
    responses:
        200:
            description: Device address successfully saved.
            type: string
        500:
            description: Internal server error
            type: string
    """
    
    ip_addr = request.remote_addr
    existing_results = DbHelper.get(quote(tango_addr))
    if len(existing_results) == 0:
        if DbHelper.add(datetime.datetime.now(), quote(tango_addr), ip_addr) == -1:
            return "Failed"
    else:
        if DbHelper.update(quote(tango_addr), datetime.datetime.now()) == -1:
            return "Failed"
    
    return "Successfully saved device address"


@app.route('/insert/<path:tango_addr>/', methods=['GET'])
def insert(tango_addr):
    """Endpoint to map a Tango device address to the sender's IP address
    ---
    tags:
      - insert
    parameters:
      - name: tango_addr
        in: path
        type: string
        required: true
    responses:
        200:
            description: Device address successfully inserted.
            type: string
        500:
            description: Internal server error
            type: string
    """
    
    ip_addr = request.remote_addr
    existing_results = DbHelper.get(quote(tango_addr))
    if len(existing_results) == 0:
        if DbHelper.add(datetime.datetime.now(), quote(tango_addr), ip_addr) == -1:
            return "Failed"
    else:
        return "Device address exist"
    
    return "Successfully inserted device address"


@app.route('/update/<path:tango_addr>/', methods=['GET'])
def update(tango_addr):
    """Endpoint to map a Tango device address to the sender's IP address
    ---
    tags:
      - update
    parameters:
      - name: tango_addr
        in: path
        type: string
        required: true
    responses:
        200:
            description: Device address successfully updated.
            type: string
        500:
            description: Internal server error
            type: string
    """
    
    ip_addr = request.remote_addr
    existing_results = DbHelper.get(quote(tango_addr))
    if len(existing_results) == 0:
        return "Device address doesn't exist"
    else:
        if DbHelper.update(quote(tango_addr), datetime.datetime.now()) == -1:
            return "Failed"
    
    return "Successfully updated device address"


@app.route('/get_addr/<path:tango_addr>/', methods=['GET'])
def get(tango_addr):
    """Endpoint to get the IP Address of a saved Tango Device
    ---
    tags:
      - get_addr
    parameters:
      - name: tango_addr
        in: path
        type: string
        required: true
    responses:
      200:
        description: JSON with the Device address, IP Address and timestamp when the device was added
        content:
            'application/json':
                schema:
                    type: object
                    properties:
                        tango_addr:
                            type: string
                            description: Tango Device Address
                        ip_addr:
                            type: string
                            description: Device IP Address
                        timestamp:
                            type: string
                            description: Time when the device was added to the database
                    example:
                        tango_addr: 'test/device/10'
                        ip_addr: '127.0.0.1'
                        timestamp: 'Sun, 17 Sep 2017 16:04:39 GMT'

    """

    results = DbHelper.get(quote(tango_addr))
    results = list(results)
    
    if len(results) == 0:
        return "Does not exist"

    return jsonify([{'tango_addr': tango_addr, 'ip_addr': e[2], 'timestamp': e[0]} for e in results])


@app.route('/add_attr/<path:tango_addr>/<string:name>/<string:attrtype>', methods=['GET'])
def add_attr(tango_addr, name, attrtype):
    """Endpoint to add attributes to saved Device
    ---
    tags:
      - add_attr
    parameters:
      - name: tango_addr
        in: path
        type: string
        required: true
      - name: name
        in: attribute name
        type: string
        required: true
      - name: attrtype
        in: attribute
        type: string
        required: true
    responses:
      200:
            description: Attribute added
            type: string
      500:
            description: Internal server error
            type: string

    """
    response = DbHelper.add_attr(tango_addr, name, attrtype)
    if response == -1:
        return "Attribute exists"
    return "Attribute added"


@app.route('/add_command/<path:tango_addr>/<string:name>/<string:cmd_params>', methods=['GET'])
def add_command(tango_addr, name, cmd_params):
    """Endpoint to add commands to saved Device
    ---
    tags:
      - add_command
    parameters:
      - name: tango_addr
        in: path
        type: string
        required: true
      - name: name
        in: command name
        type: string
        required: true
      - name: cmd_params
        in: list of command params
        type: string
        required: true
        Eg: "param1-type1,param2-type2..."
    responses:
      200:
            description: Command added
            type: string
      500:
            description: Internal server error
            type: string

    """
    cmd_arr = cmd_params.split(",")
    cmd_param_string = {}
    for cmd in cmd_arr:
        cmdparam = cmd.split("-")
        cmd_param_string[cmdparam[0]] = cmdparam[1]

    # Eg: "{"name": "type", "name": "type", "name": "type", ...}"
    response = DbHelper.add_command(tango_addr, name, json.dumps(cmd_param_string))
    if response == -1:
        return "Command exists"
    return "Command added"


def main():
    app.run(debug=True)

if __name__ == '__main__':
    main()
