import datetime
import os
import sqlite3
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
        return jsonify({})

    return jsonify([{'tango_addr': tango_addr, 'ip_addr': e[2], 'timestamp': e[0]} for e in results])

def main():
    app.run(debug=True)

if __name__ == '__main__':
    main()
