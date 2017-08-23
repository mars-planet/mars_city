import os
import threading


def func():
    os.chdir("plotly/")
    os.system("python app.py")


def funx():
    os.system("gunicorn api:app")


if __name__ == "__main__":
    os.system("sudo service mongod start")
    t1 = threading.Thread(target=funx)
    t2 = threading.Thread(target=func)
    t1.start()
    t2.start()
