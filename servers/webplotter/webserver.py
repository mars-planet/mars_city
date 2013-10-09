import re
import json
import socket
import PyTango

# give up if no answer is received after 5s
# this seems to avoid hanging of the server in case of broken pipes
socket.setdefaulttimeout(5)

devices = {}
data = []

def simple_app(environ, start_response):
    global data
    path = environ['PATH_INFO']
    m = re.match('^/([cC]3/[^/]+/[^/]+)(/json)?$', path)
    if m and m.group(2) is None:
        devname = m.group(1)
        if devname not in devices:
            device = PyTango.DeviceProxy(devname)
            devices[devname] = device
        start_response('200 OK', [('Content-Type', 'text/html')])
        with open('plot.html') as f:
            return f.read()
    elif m and m.group(2):
        devname = m.group(1)
        device = devices[devname]
        attrs = dict((attr, device[attr].value)
                     for attr in device.get_attribute_list()
                     if attr not in {'Status', 'State'})
        start_response('200 OK', [('Content-Type', 'text/json'),
                                  ('Cache-Control', 'no-cache')])
        return json.dumps(attrs)

    elif path == '/':
        start_response('200 OK', [('Content-Type', 'text/html')])
        with open('page.html') as f:
            return f.read()
    elif path == '/hunveyor':
        start_response('200 OK', [('Content-Type', 'text/json')])
        if not data:
            with open('NeuralNetwork/aoudax.data') as f:
                data = f.readlines()
                data.reverse()
        return data.pop()
    else:
        start_response('200 OK', [('Content-Type', 'text/plain')])
        return environ['PATH_INFO']


if __name__ == '__main__':
    from wsgiref.simple_server import make_server

    httpd = make_server('', 8080, simple_app)
    print "Serving on http://localhost:8080"

    httpd.serve_forever()
