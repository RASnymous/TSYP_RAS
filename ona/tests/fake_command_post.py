#!/usr/bin/env python3
"""A stand-in Command Post for tests: records every POST, serves a mission on GET."""
import json
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

POSTS = []
MISSION = {'m': None}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(n) or b'{}')
        POSTS.append((self.path, body))
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"ok":true}')

    def do_GET(self):
        if self.path.startswith('/api/mission-dispatch/latest'):
            data = json.dumps(MISSION['m']).encode()
        else:
            data = b'null'
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(data)


def serve(port):
    srv = ThreadingHTTPServer(('127.0.0.1', port), H)
    th = threading.Thread(target=srv.serve_forever, daemon=True)
    th.start()
    return srv


if __name__ == '__main__':
    serve(int(sys.argv[1]) if len(sys.argv) > 1 else 3999)
    threading.Event().wait()
