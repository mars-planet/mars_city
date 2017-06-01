from __future__ import absolute_import, division, print_function
import sys

__author__ = 'abhijith'

'''
utilityHelper provides all methods that retrieves the non-biometric data
such as user information, available users, available biometric resources, etc.
This helper module is called by the Tango server module providing a clear
line of abstraction.
'''


def allUsers():
    '''
    Param: void
    Return value: JSON response string with account information of the users
    under authenticated user's account

    Returns list of users under authenticated user's account and respective
    information such as name, email, profile and resource uri
    '''
    raise NotImplementedError


def accountInfoHelper():
    '''
    Param: void
    Return value: JSON response string with authenticated user information

    Returns only the authenticated user's data such as name, email, uri,
    profile information, etc.
    '''
    raise NotImplementedError


def userAccountInfoHelper(userID):
    '''
    Param: userID
    Return value: JSON response string with user's data whose user-id is userID

    Returns only the user's data such as name, email, uri, profile inforation,
    etc, whose user-id is userID
    '''
    raise NotImplementedError


def HexoskinDatatypesHelper(auth):
    '''
    Param: auth (authentication token)
    Return value: JSON response string containing datatypes and associated IDs

    Returns all the datatypes that the hexoskin allows. Datatypes in Hexoskin
    are the biometric resources it provides, such as Breathing rate, etc.

    Useful for adding new biometric resource support to the project.
    '''
    raise NotImplementedError


def authHelper():
    '''
    Param: void
    Return value: auth (authentication token)

    Allows you login into your hexoskin account.
    Requires the credentials mentioned below.
    It can be directly added to code (set interactiveLoginFlag = False)
    or from the terminal (set interactiveLoginFlag = True)
    '''
    raise NotImplementedError


def main(argv):
    # loginHelper()
    # getHexoskinDatatypes('dummy_auth')
    pass


if __name__ == "__main__":
    main(sys.argv)
