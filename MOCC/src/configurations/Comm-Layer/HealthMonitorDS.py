from flask import Flask, jsonify
from flasgger import Swagger


app = Flask(__name__)
app.config['SWAGGER'] = {
    'title': 'Health Monitor API',
    'uiversion': 2
}
Swagger(app)


@app.route('/health_monitor/functions/<string:fn_name>', methods=['GET'])
def execute_function(fn_name):
    """Execute a function of the Health Monitor
    ---
    tags:
      - health
      - monitor
      - execute
    parameters:
      - name: fn_name
        in: path
        type: string
        required: true
    responses:
      200:
        description: When the function is executed successfully.
        type: string
    """
    return jsonify({'status': 'SUCCESSFULLY executed ' + fn_name})


@app.route('/health_monitor/functions', methods=['GET'])
def list_functions():
    """Returns all the functions of the Health Monitor
    ---
    tags:
      - healh
      - monitor
      - function
    responses:
      200:
        description: Functions of the health monitor.
        type: JSON
    """
    functions = ['administer_insulin', 'evacuate', 'ressucitate']
    return jsonify({'functions': functions})


@app.route('/health_monitor/attributes', methods=['GET'])
def list_attributes():
    """Return all the attributes belonging to the Health Monitor
    ---
    tags:
      - health
      - monitor
    responses:
      200:
        description: Returns the attributes of the Health Monitor.
        type: JSON
    """
    attributes = ['health', 'heart_rate', 'oxygen_level']
    return jsonify({'attributes': attributes})


if __name__ == '__main__':
    app.run(debug=True)
