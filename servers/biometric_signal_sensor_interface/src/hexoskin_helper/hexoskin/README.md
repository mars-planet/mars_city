
# Hexoskin Python API Client

A Python client for accessing the Hexoskin API that provides simple, OOP-like access.

This client requires the `requests` python library.

    sudo pip install requests

Initialize a client instance like this:

    api = hexoskin.client.HexoApi('myAPIkey', 'myAPIsecret', user_auth='someuser@hexoskin.com:someuserpassword')


# Getting data

You can query the API using the list() or get() methods of a ApiResourceAccessor.  An ApiResourceAccessor is created for each endpoint on an instance of a HexoApi.

Get the current user's info

    user = api.account.list()[0]
    print user

All the users you can see:

    users = api.user.list()
    print users[0]

Passing either keyword arguments or a dictionary to list() sets the GET args of the request.  So any filtering you'd like to apply (that's supported by the API too, of course) can be managed that way.  For instance, of records before a given startTimestamp.

    records = api.record.list(startTimestamp__lt=347477726132)

Or:

    records = api.record.list({'startTimestamp__lt':347477726132})

`records` is a ApiResourceList of record ApiResourceInstances.  You can access it like a list:

    print record[0]

Note that the API resources contained in the record, such as record.user, have also been converted to ApiResourceInstances.

    print record[0].user

You can get the next page by calling next() on the list.

    records.next()

The next page will be appended to the existing list.  If there is no next page a Index error exception is raised.  You can check if there is a next page by look at the list's `nexturl`.

    if records.nexturl:
        records.next()

You may also user get() to fetch a particular resource by either URI or id.

    user99 = api.user.get(99)

    # or by URI
    ecgs = api.ecg.list()
    ecg0_user = api.user.get(ecgs[0].user)


## The DataFile Resource

The DataFile Resource works differently because it doesn't return a list of rows that you can page through, but instead a single response containing all the data that you requested.  Consequently a request to `datafile` returns an ApiDataList rather than an ApiResourceList.  You can iterate through an ApiDataList to see the ApiDataResult returned for each user (frequently this will be just the current user).  You can query it's length to see how many ApiDataResults were returned

    result = api.datafile.list(record=99999, datatype=19)
    len(result)         # -> 1

An ApiDataResult has a `user` attribute that contains the resource_uri of the user, a `record` attribute that contains the records involved in the data, and a `data` attribute that contains the returned data points.

    dataresult = result[0]
    dataresult.user     # -> '/api/v1/user/99/'
    dataresult.record   # -> an ApiResourceInstance of a Record
    dataresult.data     # -> an array of the activity data for this Record


## Creating Resources

You can create items by calling create off any ApiResourceAccessor, a Range for instance:

    new_range = api.range.create({'name':'testnew_range', 'start':353163129199, 'end':353163139199, 'user':user})

`new_range` is an ApiResourceInstance or, if a resource is not automatically returned from by the API, a string of the URI of the created resource.  You may pass the URI to api.resource_from_uri() to load the resource if desired.


## Modifying Resources

You can modify ApiResourceInstances in place and then call update():

    new_range.name = 'newtestyrangey'
    new_range.update()
    print new_range

Or by passing a dictionary to update().  Note that we're using ApiResourceInstances as a values here, regular values work too of course, using an ApiResourceInstance is just a convenience.

    new_range.update({'user': users[0]})
    print new_range


## Deleting Resources

You can delete right from the list!  Modifying our `records` ApiResourceList would send a delete request to the API except it's not allowed.

    try:
        del records[5]
    except hexoskin.errors.MethodNotAllowed, e:
        "Oh no you di'int! %s" % e

Or you can call delete() on a ApiResourceInstance.  That Range we created is probably not worth keeping, let's kill it:

    new_range.delete()


## Exceptions

There are several Exceptions defined by this library.  All but one have to do with HTTP error responses.  Here's the list:

### MethodNotAllowed

This is raised when the library notices you are trying to use a disallowed method.  No request is sent in this case.

    from hexoskin.errors import *
    try:
        api.datatype.list()[4].delete()
    except MethodNotAllowed, e:
        # handle the error somehow...

### HttpError xxx

All HTTP errors inherit from this class so you may use this to catch **all** (even ones without their own class) HTTP errors.  HttpErrors all have a ApiResponse object in `response` which you can examine for more information about the error.

    try:
        # Say you can view, but not change user 5437's annotations
        r = api.annotation.list({'user':5437})[0]
        r.update({'annotation':'I wuz here'})
    catch HttpError, e:
        print e.response


### HttpClientError 4xx
All 400-level HTTP errors inherit from this class so you may use this to catch all 400-level HTTP errors defined below.

 - **HttpBadRequest 400**
 - **HttpUnauthorized 401**
 - **HttpForbidden 403**
 - **HttpNotFound 404**
 - **HttpMethodNotAllowed 405**

### HttpServerError 5xx
All 500-level HTTP errors inherit from this class so you may use this to catch all 500 level HTTP errors defined below.

 - **HttpInternalServerError 500**
 - **HttpNotImplemented 501**


## Cached Resource List

The library derives its resource list by querying the API and stores the result in a local file.  To you can decide where this file is stored by setting the corresponding variable:

    import hexoskin.client
    hexoskin.client.CACHED_API_RESOURCE_LIST = '.api_cache'

To create the filename, the base_url has all groups of non-word chars replaced with '.' and is appended to the CACHED_API_RESOURCE_LIST value.  In code:

    cache_filename = '%s_%s' % (CACHED_API_RESOURCE_LIST, re.sub(r'\W+', '.', self.base_url))

Setting it to None will disable the caching but that's not recommended, you'll incur a pause each time a HexoApi class is initialized.  To clear the cache, either find and delete the cache file on your system, or call `clear_resource_cache()` on a HexoApi instance.  The next call that requires the resource list will refetch it from the API.

    api.clear_resource_cache()
    api.account.list() # Will refetch the resource list.

If you want to take a look at how the resource is defined (to find available filters for example), you can print it, it's just a normal dict:

    print api.resource_conf

That will likely be a little too large to be useful though.  Each resource is stored separately and links to the configs are available from all the ApiResource[type] classes.

    print api.resource_conf['range']

    # ApiResourceAccessors store a link to the config
    print api.range._conf

    # ApiResourceInstances and ApiResourceLists have a _parent to their ApiResourceAccessor
    print new_range._parent._conf
    print records._conf

You likely won't need that... but it's there!