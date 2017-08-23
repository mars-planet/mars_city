from flask import Flask, render_template
import plotly.figure_factory as ff

import json
import plotly

import pandas as pd
import numpy as np

import requests

app = Flask(__name__)
app.debug = True


@app.route('/')
def index():

    info_data = requests.get("http://localhost:8000/test").json()
    t_o_a = info_data['data']['time of arrival']
    t_o_p = info_data['time']
    alarm = info_data['data']['prediccs-alarm']
    clear_signal = info_data['data']['all-clear']
    SEP_threshold = info_data['thresholds']['SEP probability threshold']
    threshold = info_data['thresholds']
    suit_threshold = threshold['Thin spacesuit shielding threshold']
    shelter_threshold = threshold['Storm shelter shielding threshold']

    if t_o_a is None:
        t_o_a = 'None'

    if alarm == 0:
        alarm = 'None'
    else:
        alarm = 'Warning !!!'

    if clear_signal is None or clear_signal == 0:
        clear_signal = 'None'
    else:
        clear_signal = 'all-clear'

    table_data = {'data': {'time of arrival': t_o_a,
                           'alarm': alarm, 'clear': clear_signal,
                           'SEP threshold': SEP_threshold,
                           'suit threshold': suit_threshold,
                           'shelter threshold': shelter_threshold}}

    plot_data = requests.get("http://localhost:8000/plot")
    data = plot_data.json()['data']
    x2 = []
    y2 = []
    y1 = []
    x1 = []
    for i in data:
        x2.append(float(i[1]))
        y2.append(float(i[-2]))
        if float(i[-2]) > float(0.068):
            y1.append(float(i[-2]))
            x1.append(float(i[1]))

    rng = pd.date_range('1/1/2011', periods=7500, freq='H')
    ts = pd.Series(np.random.randn(len(rng)), index=rng)

    graphs = [
        dict(
            data=[
                dict(
                    x=x2,
                    y=y2,
                    name='prediccs-data',
                    type='scatter',
                    marker=dict(
                        color='green')

                ),
                dict(
                    x=x1,
                    y=y1,
                    name='above probability threshold',
                    type='scatter',
                    marker=dict(
                        color='orange')
                ),
            ],
            layout=dict(
                title='Dose Mars',
                xaxis=dict(
                    title='Coordinated Universal Time(UTC)'
                ),
                yaxis=dict(
                    title='cGy/day'
                ),
                shapes=[dict(
                    type='line',
                    x0=min(x2),
                    y0=0.068,
                    x1=max(x2),
                    y1=0.068,
                    line=dict(
                        width=1,
                        color='black',
                        opacity=0.5,
                    )

                ), ]
            ),
        )]

    # Add "ids" to each of the graphs to pass up to the client
    # for templating
    ids = ['graph-{}'.format(i) for i, _ in enumerate(graphs)]

    # Convert the figures to JSON
    # PlotlyJSONEncoder appropriately converts pandas, datetime, etc
    # objects to their JSON equivalents
    graphJSON = json.dumps(graphs, cls=plotly.utils.PlotlyJSONEncoder)
    dataJSON = json.dumps(table_data, cls=plotly.utils.PlotlyJSONEncoder)
    print dataJSON
    return render_template('layouts/index.html',
                           ids=ids,
                           graphJSON=graphJSON, data=dataJSON)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9999)
