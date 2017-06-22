import binascii, base64, cPickle, csv, hashlib, hmac, json, os, random, re, struct, time, urllib
import requests
from collections import deque
from hashlib import sha1
from urlparse import parse_qsl, urlparse
from hexoskin.errors import *


CACHED_API_RESOURCE_LIST = '.api_stash'


class ApiResourceAccessor(object):

    def __init__(self, name, conf, api):
        self._name = name
        self._conf = conf
        self.api = api


    def list(self, get_args=None, format=None, auth=None, **kwargs):
        self._verify_call('list', 'get')
        get_args = get_args or {}
        get_args.update(kwargs)
        get_args = self.api.convert_instances(get_args)
        hdrs = {'headers':{'Accept': format}} if format else {}
        response = self.api.get(self._conf['list_endpoint'], get_args, auth=auth, **hdrs)

        ctype = response.content_type

        if ctype == 'application/json':
            is_data, is_flat = self._is_data_response(response)
            if is_data:
                if is_flat:
                    return ApiFlatDataList(response, self)
                else:
                    return ApiDataList(response, self)
            else:
                return ApiResourceList(response, self)
        elif ctype == 'text/csv':
            return ApiCSVResult(response, self)
        elif ctype == 'application/octet-stream':
            return ApiBinaryResult(response, self)


    def patch(self, new_objects, auth=None, *args, **kwargs):
        self._verify_call('list', 'patch')
        return self.api.patch(self._conf['list_endpoint'], {'objects':new_objects}, auth=auth, *args, **kwargs)


    def get(self, uri, auth=None, force_refresh=False):
        self._verify_call('detail', 'get')
        if type(uri) is int or self._conf['list_endpoint'] not in uri:
            uri = '%s%s/' % (self._conf['list_endpoint'], uri)
        api_instance = self.api._object_cache.get(uri)
        if force_refresh or not api_instance or api_instance._lazy:
            response = self.api.get(uri, auth=auth)
            api_instance = self.api._object_cache.set(ApiResourceInstance(response.result, self))
        return api_instance


    def create(self, data, auth=None, *args, **kwargs):
        self._verify_call('list', 'post')
        data = self.api.convert_instances(data)
        response = self.api.post(self.endpoint, data, auth=auth, *args, **kwargs)
        if response.result:
            return self.api._object_cache.set(ApiResourceInstance(response.result, self))
        else:
            uri = response.headers['Location']
            rsrc_type,id = self.api.resource_and_id_from_uri(uri)
            return self._parent.api._object_cache.set(ApiResourceInstance({'resource_uri':v, 'id':id}, self, lazy=True))


    @property
    def endpoint(self):
        return self._conf['list_endpoint']


    def _verify_call(self, access_type, method):
        if method not in self._conf['allowed_%s_http_methods' % access_type]:
            raise MethodNotAllowed('%s method is not allowed on a %s %s' % (method, self._conf['name'], access_type))


    def _is_data_response(self, response):
        # TODO: Replace with a reasonable method of determining the response
        # type.
        is_data = isinstance(response.result, (list, basestring))
        if is_data:
            is_flat = oauth_parse_qs(response.url).get('flat', False)
            return is_data, is_flat
        return False, False



class ApiResult(object):

    def __init__(self, response, parent):
        self._parent = parent
        self.response = response



class ApiCSVResult(ApiResult, list):

    def __init__(self, response, parent):
        super(ApiCSVResult, self).__init__(response, parent)
        self.csv = csv.reader(response.result.splitlines())
        list.__init__(self, self.csv)



class ApiBinaryResult(ApiResult, bytearray):

    def __init__(self, response, parent):
        super(ApiBinaryResult, self).__init__(response, parent)
        bytearray.__init__(self, response.result)



class ApiResultList(ApiResult, deque):

    def __init__(self, response, parent):
        super(ApiResultList, self).__init__(response, parent)
        deque.__init__(self, self._make_list(response))


    def _make_list(self, response):
        return map(self._make_list_item, response.result)


    def _make_list_item(self, r):
        return r



class ApiDataList(ApiResultList):

    def _make_list_item(self, r):
        return ApiDataResult(r, self._parent)



class ApiDataResult(object):

    def __init__(self, row, parent):
        self.record = [ApiResourceInstance(r, parent.api.record) for r in row.get('record', [])]
        self.user = row['user']
        self.data = {int(d):v for d,v in row['data'].items()}



class ApiFlatDataList(ApiResultList):

    def _make_list(self, response):
        return response.result



class ApiResourceList(ApiResultList):

    def __init__(self, response, parent):
        super(ApiResourceList, self).__init__(response, parent)
        self._set_next_prev(response)


    def _make_list(self, response):
        return map(self._make_list_item, response.result['objects'])


    def _make_list_item(self, r):
        return self._parent.api._object_cache.set(ApiResourceInstance(r, self._parent))


    def __delitem__(self, key):
        self[key].delete()
        return super(ApiResourceList, self).__delitem__(key)


    def load_next(self):
        if self.nexturl:
            response = self._parent.api.get(self.nexturl)
            self._append_response(response)
        else:
            raise StopIteration('List is already at the end.')


    def load_prev(self):
        if self.prevurl:
            response = self._parent.api.get(self.prevurl)
            self._append_response(response, prepend=True)
        else:
            raise StopIteration('List is already at the beginning.')


    def _append_response(self, response, prepend=False):
        try:
            self._set_next_prev(response)
            if prepend is True:
                self.extendleft(self._make_list(response))
            else:
                self.extend(self._make_list(response))
        except KeyError, e:
            raise ApiError('Cannot parse results, unexpected content received! %s \nFirst 64 chars of content: %s' % (e, response.body[:64]))


    def _set_next_prev(self, response):
        self.nexturl = response.result['meta'].get('next', None)
        self.prevurl = response.result['meta'].get('prev', None)



class ApiResourceInstance(object):

    def __init__(self, obj, parent, lazy=False):
        self.__dict__['fields'] = {}
        self._lazy = lazy
        self._parent = parent
        self.update_fields(obj)


    def update_fields(self, obj):
        # Skip __setattr__ for this one. Should we derive from parent._conf.fields instead?
        self.__dict__['fields'] = obj
        self._link_instances()


    def _link_instances(self):
        # Loop through the fields populating foreign keys.
        for k,v in self.fields.items():
            if k in self._parent._conf['fields'] and self._parent._conf['fields'][k].get('related_type', None) == 'to_one':
                if isinstance(v, dict):
                    rsrc_type,id = self._parent.api.resource_and_id_from_uri(v.get('resource_uri', ''))
                    if rsrc_type:
                        self.fields[k] = self._parent.api._object_cache.set(ApiResourceInstance(v, rsrc_type))

                elif isinstance(v, basestring):
                    rsrc_type,id = self._parent.api.resource_and_id_from_uri(v)
                    if rsrc_type:
                        # Is there already a cached object?
                        rsrc = self._parent.api._object_cache.get(v)
                        # If not, create a lazy one.
                        if not rsrc:
                            rsrc = self._parent.api._object_cache.set(ApiResourceInstance({'resource_uri':v, 'id':id}, rsrc_type, lazy=True))
                        self.fields[k] = rsrc


    def __getattr__(self, name):
        if name in self.fields:
            # Special case to decode data fields.
            if name == 'data':
                return self._decode_data()
            return self.fields[name]
        elif self._lazy and 'resource_uri' in self.fields:
            self._parent.api.resource_from_uri(self.fields['resource_uri'])
            self._lazy = False
            return getattr(self, name)
        raise AttributeError("Attribute '%s' not found on %s" % (name, self._parent._conf['name']))


    def __setattr__(self, name, value):
        if name in self.fields:
            self.fields[name] = value
        else:
            super(ApiResourceInstance, self).__setattr__(name, value)


    def __repr__(self):
        # Lame exception for devices.  :(
        pk = 'deviceid' if self._parent._name == 'device' else 'id'
        return '<%s.%s: %s>' % (self.__module__, self._parent._name, getattr(self, pk, None))


    def update(self, data=None, *args, **kwargs):
        self._parent._verify_call('detail', 'put')
        if data is not None:
            for k,v in data.items():
                setattr(self, k, v)
        response = self._parent.api.put(self.fields['resource_uri'], self._parent.api.convert_instances(self.fields), *args, **kwargs)
        if response.result:
            self.update_fields(response.result.copy())
        return response


    def delete(self, *args, **kwargs):
        self._parent._verify_call('detail', 'delete')
        response = self._parent.api.delete(self.fields['resource_uri'], *args, **kwargs)
        self.fields = {k: None for k in self.fields.keys()}


    def _decode_data(self):
        if not hasattr(self, '_decoded_data'):
            self._decoded_data = None
            for fn in (self._decode_binary, self._decode_array):
                try:
                    self._decoded_data = fn(self.fields['data'])
                    break
                except Exception:
                    pass
        return self._decoded_data


    def _decode_binary(self, data):
        return struct.unpack('i' * self.nsample, base64.b64decode(data))


    def _decode_array(self, data):
        return [tuple(int(i) for i in v.split(',')) for v in data.strip('()[]').split('), (')]



class ApiHelper(object):

    def __init__(self, api_key=None, api_secret=None, api_version='', auth=None, base_url=None):
        super(ApiHelper, self).__init__()
        self.resource_conf = {}
        self.resources = {}
        self._resource_cache = None
        self._object_cache = ApiObjectCache(self)

        self.api_key = api_key
        self.api_secret = api_secret
        self.api_version = api_version
        self.auth = self._create_auth(auth, key=api_key, secret=api_secret)
        self.base_url = self._parse_base_url(base_url)

        if CACHED_API_RESOURCE_LIST is not None:
            self._resource_cache = ('%s_%s' % (CACHED_API_RESOURCE_LIST, re.sub(r'\W+', '.', '%s:%s' % (self.base_url, self.api_version)))).rstrip('.')


    def __getattr__(self, name):
        if len(self.resources) == 0:
            self.build_resources()
        if name in self.resources:
            return self.resources[name]
        if name in self.resource_conf:
            self.resources[name] = ApiResourceAccessor(name, self.resource_conf[name], self)
            return self.resources[name]
        else:
            raise AttributeError("'%s' is not a valid API endpoint" % name)


    def clear_resource_cache(self):
        if self._resource_cache is not None:
            if os.path.isfile(self._resource_cache):
                os.remove(self._resource_cache)
                self.resources = {}
                self.resource_conf = {}


    def clear_object_cache(self):
        self._object_cache.clear()


    def build_resources(self):
        if self._resource_cache is not None:
            try:
                with open(self._resource_cache, 'r') as f:
                    self.resource_conf = cPickle.load(f)
            except IOError:
                self._fetch_resource_list()
                try:
                    with open(self._resource_cache, 'w+') as f:
                        cPickle.dump(self.resource_conf, f)
                except IOError, e:
                    print "Couldn't write to stash file: %s" % e
        else:
            self._fetch_resource_list()


    def _create_auth(self, auth, key=None, secret=None):
        if not auth:
            return None
        elif isinstance(auth, (requests.auth.HTTPBasicAuth, HexoAuth, OAuth1Token, OAuth2Token)):
            return auth
        elif isinstance(auth, basestring):
            return HexoAuth(key, secret, *auth.split(':'))
        elif len(auth) == 2:
            return HexoAuth(key, secret, *auth)
        else:
            return None


    def _fetch_resource_list(self):
        resource_list = self.get('/api/').result
        for n,r in resource_list.iteritems():
            if n == 'import':
                continue
            self.resource_conf[n] = self.get(r['schema']).result
            self.resource_conf[n]['list_endpoint'] = r['list_endpoint']
            self.resource_conf[n]['name'] = n
            time.sleep(.3)


    def _parse_base_url(self, base_url):
        parsed = urlparse(base_url)
        if parsed.netloc:
            return 'https://' + parsed.netloc
            # return 'http://' + parsed.netloc
        raise ValueError('Unable to determine URL from provided base_url arg: %s.', base_url)


    def convert_instances(self, value_dict):
        """
        Converts all ApiResourceInstances into their uri_resource equivilant.
        Since we don't update child properties, this makes everything work
        more smoothly when sending data to the API.
        """
        return {k: v.resource_uri if k in self.resources and type(v) is ApiResourceInstance else v for k,v in value_dict.items()}


    def _request(self, path, method, data=None, params=None, auth=None, headers=None):
        auth = self._create_auth(auth) if auth else self.auth
        if data and not isinstance(data, basestring):
            data = json.dumps(data)
        if params:
            # Make lists or sets comma-separated strings.
            params = {k:','.join(str(i) for i in v) if isinstance(v, (tuple, list)) else v for k,v in params.items()}
        url = self.base_url + path
        req_headers = {'Accept': 'application/json', 'Content-type': 'application/json'}
        if self.api_version:
            req_headers['X-HexoAPIVersion'] = self.api_version
        if headers:
            req_headers.update(headers)
        response = ApiResponse(requests.request(method, url, data=data, params=params, headers=req_headers, auth=auth, verify=False), method)
        if response.status_code >= 400:
            self._throw_http_exception(response)
        return response


    def post(self, path, data=None, auth=None, headers=None):
        return self._request(path, 'post', data, auth=auth, headers=headers)


    def get(self, path, data=None, auth=None, headers=None):
        return self._request(path, 'get', params=data, auth=auth, headers=headers)


    def put(self, path, data=None, auth=None, headers=None):
        return self._request(path, 'put', data, auth=auth, headers=headers)


    def patch(self, path, data=None, auth=None, headers=None):
        return self._request(path, 'patch', data, auth=auth, headers=headers)


    def delete(self, path, auth=None, headers=None):
        return self._request(path, 'delete', auth=auth, headers=headers)


    def resource_from_uri(self, path):
        if path:
            if path.startswith(self.base_url):
                path = path[len(self.base_url):]
            rsrc_type,id = self.resource_and_id_from_uri(path)
            if rsrc_type:
                return rsrc_type.get(path)
        return None


    def resource_and_id_from_uri(self, path):
        base_uri,id = re.match('^(.+?)(\d+)/$', path).groups()
        for k,r in self.resource_conf.items():
            if r['list_endpoint'] == base_uri:
                return getattr(self, k), id
        return None, None


    def _throw_http_exception(self, response):
        if response.status_code == 400:
            raise HttpBadRequest(response)
        if response.status_code == 401:
            raise HttpUnauthorized(response)
        if response.status_code == 403:
            raise HttpForbidden(response)
        if response.status_code == 404:
            raise HttpNotFound(response)
        if response.status_code == 405:
            raise HttpMethodNotAllowed(response)
        if response.status_code == 500:
            raise HttpInternalServerError(response)
        if response.status_code == 501:
            raise HttpNotImplemented(response)
        raise HttpError(response)


    def oauth1_get_request_token_url(self, callback_uri):
        self.auth = OAuth1Token(self.api_key, self.api_secret, oauth_callback=callback_uri)
        # oauth_header = create_oauth_header(oauth, 'POST', request_token_url, secret)
        # return requests.post(request_token_url, headers={'Authorization': oauth_header})
        resp = self.post('/oauth/request_token')
        tokens = oauth_parse_qs(resp.content)
        self.auth.set(**tokens)
        return '%s/oauth/authorize?oauth_token=%s' % (self.base_url, tokens['oauth_token'])


    def oauth1_get_access_token(self, url):
        tokens = oauth_parse_qs(url)
        self.auth.oauth_verifier = tokens['oauth_verifier']
        resp = self.post('/oauth/access_token')
        self.auth.set(**oauth_parse_qs(resp.content))
        return self.auth


    def oauth2_get_request_token_url(self, callback_uri, grant_type='authorization_code', scope='readonly'):
        self.auth = OAuth2Token(self.api_key, self.api_secret)
        self.auth.callback_uri = callback_uri
        self.auth.grant_type = grant_type
        self.auth.scope = scope
        get_args = {
            'response_type': self.auth.response_type,
            'state': self.auth.generate_state(),
            'client_id': self.api_key,
            'scope': self.auth.scope,
            'redirect_uri': self.auth.callback_uri,
        }
        querystr = '&'.join('%s=%s' % (k, oauth_encode(v)) for k,v in get_args.items())
        return '%s/api/connect/oauth2/auth/?%s' % (self.base_url, querystr)


    def oauth2_get_access_token(self, *args, **kwargs):
        """
        Gets the access token from the URL returned by an oauth2 user
        authentication or from a user/pass combination for a password
        grant_type client.
        """
        if len(args) == 1:
            url = args[0]
            bits = oauth_parse_qs(url, fragment=self.auth.response_type == 'token')
            if self.auth.state != bits['state']:
                raise Exception("State verification failed! Couldn't find %s in %s" % (self.auth.state, bits))
            if self.auth.response_type == 'code':
                return self._fetch_oauth2_access_token(
                    grant_type='authorization_code',
                    code=bits['code'],
                    redirect_uri=self.auth.callback_uri
                )
            elif self.auth.response_type == 'token':
                self.auth.set(**bits)
                return self.auth
        elif len(args) == 2:
            kwargs.update(zip(('username', 'password'), args))
            self.auth = OAuth2Token(self.api_key, self.api_secret)
            return self._fetch_oauth2_access_token(grant_type='password', **kwargs)
        else:
            raise ValueError('Unexpected arguments passed to oauth2_get_access_token()')


    def _fetch_oauth2_access_token(self, **kwargs):
        basicauth = requests.auth.HTTPBasicAuth(self.api_key, self.api_secret)
        # response = self.post('/api/connect/oauth2/token/', auth=basicauth, data=kwargs)
        response = requests.post('%s/api/connect/oauth2/token/' % self.base_url, data=kwargs, auth=basicauth)
        if response.status_code >= 400:
            self._throw_http_exception(response)
        self.auth.set(**response.json())
        return self.auth



class HexoAuth(requests.auth.HTTPBasicAuth):
    """
    Supports BasicAuth and Hexo signatures.
    """

    def __init__(self, api_key, api_secret, auth, password=None):
        self.auth = auth
        self.api_key = api_key
        self.api_secret = api_secret
        if not password:
            self.username, self.password = auth.split(':')
        else:
            self.username = auth
            self.password = password


    def __call__(self, request):
        request = super(HexoAuth, self).__call__(request)
        ts = int(time.time())
        digest = hashlib.sha1('%s%s%s' % (self.api_secret, ts, request.url)).hexdigest()
        request.headers['X-HEXOTIMESTAMP'] = ts
        request.headers['X-HEXOAPIKEY'] = self.api_key
        request.headers['X-HEXOAPISIGNATURE'] = digest
        # print '(%s, %s, %s) = %s' % (self.api_secret, ts, request.url, digest)
        return request



class OAuth1Token(object):
    """
    Basic OAuth1 support in combination with ApiHelper.
    """

    __slots__ = (
        'oauth_consumer_key',
        'oauth_consumer_secret',
        'oauth_callback',
        'oauth_token',
        'oauth_token_secret',
        'oauth_authorized_realms',
        'oauth_verifier',
        'oauth_callback_confirmed',
    )

    _request_keys = ('oauth_callback', 'oauth_token', 'oauth_token_secret', 'oauth_verifier')


    def __init__(self, oauth_consumer_key, oauth_consumer_secret, **kwargs):
        self.oauth_consumer_key = oauth_consumer_key
        self.oauth_consumer_secret = oauth_consumer_secret
        self.set(**kwargs)


    def set(self, **kwargs):
        for k,v in kwargs.items():
            setattr(self, k, v)


    def _request_args(self):
        return dict(filter(lambda kv:kv[1] is not None, [(k, getattr(self, k, None)) for k in self._request_keys]))


    def __call__(self, request):
        req_params = oauth_parse_qs(request.url)
        oauth_vars = self._request_args()
        token_secret = oauth_vars.pop('oauth_token_secret', '')
        key = '&'.join(oauth_encode(i) for i in [self.oauth_consumer_secret, token_secret])

        oauth_vars['oauth_consumer_key'] = self.oauth_consumer_key
        oauth_vars['oauth_signature_method'] = 'HMAC-SHA1'
        oauth_vars['oauth_nonce'] = random.randint(1000000,9999999)
        oauth_vars['oauth_timestamp'] = int(time.time())

        oauth_vars = {oauth_encode(k): oauth_encode(str(v)) for k,v in oauth_vars.items()}

        params = oauth_vars.copy()
        params.pop('realms', None)
        params.update(req_params or {})
        params_str = '&'.join('%s=%s'%(k,v) for k,v in sorted(params.items()))

        qpos = request.url.rfind('?')
        uri = request.url[:qpos if qpos > -1 else len(request.url)]
        base_str = '&'.join(oauth_encode(i) for i in [request.method.upper(), uri, params_str])

        oauth_vars['oauth_signature'] = oauth_encode(binascii.b2a_base64(hmac.new(key, base_str, sha1).digest())[:-1])
        # print ' key: %s' % key
        # print ' params: %s' % params
        # print ' base_str: %s' % base_str
        # print ' sig: %s' % oauth_vars['oauth_signature']

        request.headers['Authorization'] = 'OAuth ' + ','.join('%s="%s"'%(k,v) for k,v in oauth_vars.items())
        return request



class OAuth2Token(object):
    """
    Basic OAuth2 support in combination with ApiHelper.
    """

    __slots__ = (
        '_grant_type',
        'access_token',
        'callback_uri',
        'expires_in',
        'key',
        'refresh_token',
        'response_type',
        'scope',
        'secret',
        'state',
        'token_type',
    )


    def __init__(self, key, secret, **kwargs):
        self.key = key
        self.secret = secret


    def __call__(self, request):
        request.headers['Authorization'] = 'Bearer %s' % self.access_token
        return request


    def generate_state(self):
        self.state = str(random.randint(1000000,9999999))
        return self.state


    def set(self, **kwargs):
        for k,v in kwargs.items():
            setattr(self, k, v)


    @property
    def grant_type(self):
        return self._grant_type


    @grant_type.setter
    def grant_type(self, val):
        if val == 'authorization_code':
            self.response_type = 'code'
        elif val == 'implicit':
            self.response_type = 'token'
        elif val != 'password':
            raise ValueError('Invalid or unsupported grant_type specified.')
        self._grant_type = val



class HexoApi(ApiHelper):

    def __init__(self, api_key, api_secret, api_version='', auth=None, base_url=None):
        if base_url is None:
            base_url = 'https://api.hexoskin.com'
        return super(HexoApi, self).__init__(api_key, api_secret, api_version, auth, base_url)



class ApiResponse(object):
    """
    This was built before using the excellent `requests` library so now it
    seems a little silly to wrap the requests.response object with a less 
    functional one.  It's here for compatibility now.  TODO: remove.
    """

    def __init__(self, response, method='GET'):
        try:
            self.result = response.json()
        except:
            self.result = response.content
        self.body = response.content
        self.url = response.request.url
        self.method = method.upper()
        self.response = response

    def success(self):
        200 <= self.status_code < 400


    @property
    def content_type(self):
        return self.headers.get('content-type', '').split(';')[0]


    def __getattr__(self, attr):
        return getattr(self.response, attr)


    def __str__(self):
        return '%s %s %s\n%s' % (self.status_code, self.method.ljust(6), self.url, self.result)



class ApiObjectCache(object):

    def __init__(self, api, ttl=3600):
        self.api = api
        self.ttl = ttl
        self._objects = {}
        self._keys = self._objects.viewkeys()


    def get(self, uri):
        uri = self._strip_host(uri)
        obj = self._objects.get(uri, None)
        if obj:
            if time.time() - obj[0] < self.ttl:
                return obj[1]
            else:
                del self._objects[uri]
        return None


    def set(self, obj):
        uri = obj.resource_uri
        if uri in self._objects:
            self._objects[uri][1].update_fields(obj.fields)
            return self._objects[uri][1]
        else:
            self._objects[uri] = (time.time(), obj)
            return obj


    def clear(self, uri):
        uri = self._strip_host(uri)
        if uri in self._objects:
            del self._objects[uri]


    def _strip_host(self, uri):
        if uri.startswith(self.api.base_url):
            uri = uri[len(self.api.base_url):]
        return uri



def oauth_parse_qs(url, fragment=False):
    """
    Accepts either an URL or just the query string, or optionally will look
    only in the fragment.
    """
    bits = urlparse(url)
    return dict(parse_qsl(bits.fragment if fragment else bits.query or bits.path))


def oauth_encode(val):
    return urllib.quote(val, '-._~')
