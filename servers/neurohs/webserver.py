from subprocess import Popen, PIPE

def simple_app(environ, start_response):
    path = environ['PATH_INFO']
    if path == '/':
        start_response('200 OK', [('Content-Type', 'text/html')])
        with open('eegpoints.html') as f:
            return f.read()
    elif path == '/eegdata':
        start_response('200 OK', [('Content-Type', 'text/json')])
        return headset.stdout.readline().strip()
    elif path == '/eegpoints.gif':
        # this is not currently used
        start_response('200 OK', [('Content-Type', 'image/gif')])
        with open('eegpoints.gif') as f:
            return f.read()

if __name__ == '__main__':
    from wsgiref.simple_server import make_server

    print "Setting up headset (this might take a few seconds)...",
    headset = Popen(['python', 'neurohs.py', '500'],
                    stdout=PIPE, stderr=PIPE)
    line = None
    while line != '-----':
        line = headset.stdout.readline().strip()

    print "[OK]"

    httpd = make_server('', 8080, simple_app)
    print "Serving on http://localhost:8080/"

    httpd.serve_forever()
