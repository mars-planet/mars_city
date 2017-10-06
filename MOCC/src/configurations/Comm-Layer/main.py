import datetime
import os
import sqlite3
from urllib.parse import quote_plus

from flasgger import Swagger
from flask import Flask, jsonify, request

if 'tango.db' not in os.listdir('.'):
    conn = sqlite3.connect('tango.db', check_same_thread=False)
    lookup_table = conn.cursor()
    lookup_table.execute('''CREATE TABLE lookup(timestamp datetime,tango_addr varchar,ip_addr varchar)''')
    conn.commit()
    conn.close()

app = Flask(__name__)
swagger = Swagger(app)

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
    conn = sqlite3.connect('tango.db', check_same_thread=False)
    lookup_table = conn.cursor()
    ip_addr = request.remote_addr
    try:
        values = (datetime.datetime.now(), quote_plus(tango_addr), ip_addr)
        lookup_table.execute('INSERT INTO lookup VALUES (?,?,?)', values)
        conn.commit()
    except Exception as e:
        print(e)
        conn.close()
        return "Failed"
    conn.close()
    return "Successfully saved device address"

@app.errorhandler(500)
def internal_error(error):
    return "HTTP 500: Internal Server Error"

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
    conn = sqlite3.connect('tango.db', check_same_thread=False)
    lookup_table = conn.cursor()
    tango_addr = (quote_plus(tango_addr), )
    results = lookup_table.execute("SELECT * FROM lookup WHERE tango_addr=?", tango_addr)
    results = list(results)
    if len(results) == 0:
        conn.close()
        return jsonify({})
    conn.close()
    return jsonify([{'tango_addr': tango_addr[0], 'ip_addr': e[2], 'timestamp': e[0]} for e in results])

if __name__ == '__main__':
    app.run(debug=True)
