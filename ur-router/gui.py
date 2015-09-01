#!/usr/bin/env python
import sys
import os
import subprocess
import fcntl
import signal
from PyQt4.QtCore import *
from PyQt4.QtGui import *

class RouterDlg(QDialog):
    def __init__(self):
        super(QDialog, self).__init__()
        self.proc = None

        layout = QVBoxLayout(self)
        button_layout = QHBoxLayout()
        self.btn_start_tcp = QPushButton("Start TCP Router")
        self.btn_start_udp = QPushButton("Start UDP Router")
        self.btn_stop = QPushButton("Stop Router")
        self.btn_stop.setEnabled(False)
        self.btn_close = QPushButton("Close Window")
        button_layout.addWidget(self.btn_start_tcp)
        button_layout.addWidget(self.btn_start_udp)
        button_layout.addWidget(self.btn_stop)
        button_layout.addWidget(self.btn_close)
        self.browser = QTextBrowser()
        layout.addLayout(button_layout)
        layout.addWidget(self.browser)
        self.connect(self.btn_close, 
                     SIGNAL("clicked()"), 
                     self._cb_close)
        self.connect(self.btn_start_udp, 
                     SIGNAL("clicked()"), 
                     self.start_udp)
        self.connect(self.btn_start_tcp, 
                     SIGNAL("clicked()"), 
                     self.start_tcp)
        self.connect(self.btn_stop, 
                     SIGNAL("clicked()"), 
                     self._cb_stop)
        self.timer = QTimer()
        self.connect(self.timer, 
                     SIGNAL("timeout()"), 
                     self.timeout)

    def _cb_stop(self):
        if self.proc is None or self.proc.poll() is not None:
                self.browser.append("Router not running")
                return
        pid = self.proc.pid
        os.system('pkill -15 ^router')
        self.browser.append("Stopped PID %d"%pid)
        

    def _cb_close(self):
        self.accept()

    def start_udp(self):
        self.start_cmd("/root/ur-router/startrouter_udp.sh")

    def start_tcp(self):
        self.start_cmd("/root/ur-router/startrouter_tcp.sh")

    def start_cmd(self, cmd):
        self.cmd = cmd
        if self.proc is not None:
            if self.proc.poll() is None:
                self.browser.append("Router already running")
        p_env = os.environ
        p_env["PYTHONUNBUFFERED"] = "1"
        self.proc = subprocess.Popen(self.cmd, 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE,
                                     shell=False,
                                     env=p_env)
        fd = self.proc.stdout.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl|os.O_NONBLOCK)
        fd = self.proc.stderr.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl|os.O_NONBLOCK)
        self.timer.start(100)
        self.btn_start_udp.setEnabled(False)
        self.btn_start_tcp.setEnabled(False)
        self.btn_close.setEnabled(False)
        self.btn_stop.setEnabled(True)

    def timeout(self):
        while True:
            try:
                out = self.proc.stdout.readline()
                if out == "":
                    break
                self.browser.append(out[:-1])
            except:
                break
        while True:
            try:
                out = self.proc.stderr.readline()
                if out == "":
                    break
                self.browser.append(out[:-1])
            except:
                break
        if self.proc.poll() is not None:
            self.browser.append("Router Stopped. Code %d"%self.proc.poll())
            self.timer.stop()
            self.btn_start_udp.setEnabled(True)
            self.btn_start_tcp.setEnabled(True)
            self.btn_close.setEnabled(True)
            self.btn_stop.setEnabled(False)


class UrGui(QMainWindow):
    def __init__(self, *args):
        apply(QMainWindow.__init__, (self,) + args)
        self.button = QPushButton("Router")
        self.setCentralWidget(self.button)
        self.resize(100, 30)
        self.connect(self.button, SIGNAL("clicked()"), self._cb_click)
        self.setWindowState(Qt.WindowActive)
        self.timer = QTimer()
        self.connect(self.timer, 
                     SIGNAL("timeout()"), 
                     self.timeout)
        self.timer.start(1000)

    def _cb_click(self):
        dlg = RouterDlg()
        dlg.exec_()

    def timeout(self):
        self.setWindowState(Qt.WindowActive)
        self.raise_()
        

def main(args):
    app = QApplication(args)
    gui = UrGui()
    gui.show()
    app.exec_()

if __name__=="__main__":
    main(sys.argv)



