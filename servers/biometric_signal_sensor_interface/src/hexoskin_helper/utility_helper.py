from __future__ import absolute_import, division, print_function
import sys
import datetime
import hexoskin.client
import hexoskin.errors
import requests
import ConfigParser

requests.packages.urllib3.disable_warnings()
config = ConfigParser.ConfigParser()
config.read("../health_monitor/login_config.cfg")

__author__ = 'abhijith'

'''
utilityHelper provides all methods that retrieves the non-biometric data
such as user information, available users, available biometric resources, etc.
This helper module is called by the Tango server module providing a clear
line of abstraction.
'''

# Model type : Hexoskin or CHA3000
MODEL = 'Hexoskin'

# Specify timestamp output format : Epoch or String. Epoch goes for the
# standard Hexoskin epoch, while String is human-formatted string
TIMESTAMP = 'Epoch'

# Timestamp format
# Use only if TIMESTAMP is set to String
TIMESTAMP_FORMAT = '%Y:%m:%d\t%H:%M:%S:%f'

# Datatypes definitions
if MODEL == 'Hexoskin':
    raw_datatypes = {'acc': [4145, 4146, 4147],
                     'ecg': [4113],
                     'resp': [4129, 4130]}
    datatypes = {'activity': [49],
                 'cadence': [53],
                 'heartrate': [19],
                 'minuteventilation': [36],
                 'vt': [37],
                 'breathingrate': [33],
                 'hr_quality': [1000],
                 'br_quality': [1001],
                 'rrintervalstatus': [1004],
                 'inspiration': [34],
                 'expiration': [35],
                 'batt': [247],
                 'step': [52],
                 'rrinterval': [18],
                 'qrs': [22],
                 }

#  Sample rates definitions (in 256/samples/second)
dataSampleRate = {
    4145: 4,  # 64Hz
    4146: 4,
    4147: 4,
    4113: 1,  # 256Hz
    4114: 1,
    4115: 1,
    4129: 2,  # 128Hz
    4130: 2,
    81: 256,  # 1Hz
    49: 256,
    53: 256,
    19: 256,
    36: 256,
    37: [],
    33: 256,
    1000: 256,
    1001: 256,
    1002: 256,
    66: 256,
    98: 256,
    34: 256,
    35: 256,
    247: 256,
    52: [],
    18: 256,
    1004: 256,  # actually its async
    22: 256,
    212: 256,
    97: 256,
    208: 256
}


class SessionInfo:
    """
    This is the class containing your api login information. Instantiate an
    object of this class and pass it as argument
    to the api calls you make. You can instantiate as many tokens as you want
    """

    def __init__(self, publicKey='null', privateKey='null', username='null',
                 password='null', base_url='api'):
        """
        The init function instantiate your login token. Your API calls will
        need this to authenticate to our servers
            @param username :   The username to use for creating the token.
                                This will influence what you can and can not
                                see
            @param password :   password to login in the "username" account
            @param database :   The database to log into. Choices are api for
                                the production database, or sapi for the
                                development database
        """
        if base_url == 'api':
            apiurl = 'https://api.hexoskin.com'
        elif base_url == 'sapi':
            apiurl = 'https://sapi.hexoskin.com'
        elif base_url == 'dapi':
            apiurl = 'https://dapi.hexoskin.com'
        elif base_url == 'lapi':
            apiurl = 'https://lapi.hexoskin.com:4433'
        elif base_url == 'dapi':
            apiurl = 'https://dapi.hexoskin.com'
        else:
            raise NotImplementedError
        print("Fetching...")
        self.api = hexoskin.client.HexoApi(
            publicKey, privateKey, base_url=apiurl, auth=username + ':' +
            password, api_version='3.3.x')
        authCode = test_auth(self.api)
        if authCode != '':
            print("Failed...")
            raise


def auth_login():
    '''
    Allows you login into your hexoskin account.
    Requires the credentials mentioned below.
        @return :   auth (authentication token)
    '''
    # Credentials should be added in ../biometric_monitor/login_config.cfg
    username = config_helper("Credentials")['username']
    password = config_helper("Credentials")['password']
    publicKey = config_helper("Credentials")['publickey']
    privateKey = config_helper("Credentials")['privatekey']
    try:
        if (publicKey == '' or privateKey == '' or
                privateKey == '' or privateKey == ''):
            raise Exception("Credentials missing")

    except Exception as error:
        error_msg = error.args
        print(error_msg[0])

    else:
        auth = SessionInfo(publicKey=publicKey, privateKey=privateKey,
                           username=username, password=password)

        return auth


def test_auth(api):
    '''
    Tests whether the login to hexoskin servers was successful or not.
    Called by auth_login()
    Requires the credentials mentioned below.
        @return : empty string (if valid) or 'login_invalid'
    '''
    try:
        api.account.list()
    except Exception, e:
        if e.response.result == '':
            return 'login_invalid'
        elif e.response.result['error'] == 'API signature failed.':
            return 'key_invalid'
    return ''


def all_users(auth):
    '''
    Returns list of users under authenticated user's account and respective
    information such as name, email, profile and resource uri
        @param auth :   authentication token
        @return :       JSON response string with account information
                        of the users under authenticated user's account
    '''
    users = auth.api.user.list()
    return users.response


def account_info_helper(auth):
    '''
    Returns only the authenticated user's data such as name, email, uri,
    profile information, etc.
        @param auth :   authentication token
        @return :       JSON response string with authenticated user
                        information
    '''
    users = auth.api.account.list()
    return users.response


def user_account_info_helper(auth, userID):
    '''
    Param: userID
    Return value: JSON response string with user's data whose user-id is userID

    Returns only the user's data such as name, email, uri, profile inforation,
    etc, whose user-id is userID
        @param auth :   authentication token
        @param userID : user id
        @return :       JSON response string with user's data
                        whose user-id is userID
    '''
    raise NotImplementedError


def convertTimestamps(arr, format):
    '''
    Converts timestamps from hexoskin timestamp format to human readable format
    Called by other functions
        @param arr :        timestamp
        @param format :     desirec output format
        @return :           timestamp in format
    '''
    if format == 'Epoch':
        out = arr
    if format == 'String':
        out = []
        for i in arr:
            if len(arr[0]) == 1:
                ts = datetime.datetime.fromtimestamp(
                    float(i) / 256).strftime(TIMESTAMP_FORMAT)
                out.append(ts)
            elif len(arr[0]) > 1:
                ts = datetime.datetime.fromtimestamp(
                    float(i[0]) / 256).strftime(TIMESTAMP_FORMAT)
                line = []
                line.append(ts)
                [line.append(x) for x in i[1:]]
                out.append(tuple(line))
    return out


def clearCache(auth):
    """
    This function clears the API cache. To be used only if the resource
    list changes, which shouldn't happen often
        @param auth :   authentication token
    """
    auth.api.clear_resource_cache()


def config_helper(section):
    '''
    Returns a dictonary of the configuration stored in
    ../biometric_monitor/config.cfg
        @param section: configuration section from the config file that,
                        has to be read
    '''
    dict_config = {}
    options = config.options(section)
    for option in options:
        try:
            dict_config[option] = config.get(section, option)
        except:
            print("exception on %s!" % option)
            dict_config[option] = None
    return dict_config


def main(argv):
    pass


if __name__ == "__main__":
    main(sys.argv)
