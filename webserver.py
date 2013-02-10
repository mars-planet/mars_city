data = []

def simple_app(environ, start_response):
    global data
    path = environ['PATH_INFO']
    if path == '/':
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
