import pandas as pd
import requests
import requests_ftp


def retrieve_data(dest_dir):
    """Retrieve the data from the FTP source."""
    data_url = 'ftp://ftp.swpc.noaa.gov/pub/indices/old_indices/{}_DSD.txt'

    # ftp plugin for requests
    requests_ftp.monkeypatch_session()
    session = requests.Session()

    # Save the content in a list, in order to create the csv files
    ftp_content = []
    for year in range(1994, 2015):
        response = session.get(data_url.format(year))

        if response.ok:
            ftp_content.append(response.content)

    #TODO: Save in directory (pandas)


def parse_data(source_dir):
    """Parse the data from a directory to a single csv file."""
    pass
