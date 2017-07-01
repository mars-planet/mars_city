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
    # TODO Make function calls here
    # HOW?
    return jsonify({'status': 'SUCCESSFULLY executed ' + fn_name})


@app.route('/health_monitor/functions', methods=['GET'])
def list_functions():
    '''
    return all the functions that the Health Monitor has
    '''
    functions = ['administer_insulin', 'evacuate', 'ressucitate']
    return jsonify({'functions': functions})


@app.route('/health_monitor/attributes', methods=['GET'])
def list_attributes():
    '''
    return all the attributes that the Health Monitor has
    '''
    attributes = ['health', 'heart_rate', 'oxygen_level']
    return jsonify({'attributes': attributes})


if __name__ == '__main__':
    app.run(debug=True)
