#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paramiko
# from paramiko import SSHClient
from scp import SCPClient
# import ssh
from stat import S_ISDIR, S_ISREG


LOG_DIR = "/home/augbooth/catkin_ws/local/roslogs/latest/"
USER = "augbooth"
PW = "aug"
CALI_LOG = "calibration"


class Get_file():
    def __init__(self, hostname):
        print("connecting to {}".format(hostname))
        self.ssh_ob = paramiko.SSHClient()
        self.ssh_ob.load_system_host_keys()
        self.ssh_ob.connect(hostname, username=USER,password=PW)
        self.sftp = self.ssh_ob.open_sftp()
        self.scp = SCPClient(self.ssh_ob.get_transport())


    def get_calibration_file_name(self):
        # ssh_ob = paramiko.SSHClient()
        # ssh_ob.load_system_host_keys()
        # ssh_ob.connect(hostname, username=USER, password=PW)
        # sftp = ssh_ob.open_sftp()

        for entry in self.sftp.listdir_attr(LOG_DIR):
            # mode = entry.st_mode
            # if S_ISDIR(mode):
            #     print(entry.filename + " is folder")
            # elif S_ISREG(mode):
            #     print(entry.filename + " is file")

            if entry.filename.startswith(CALI_LOG):
                res = entry.filename
        return res

    def get_file(self, file_name):

        # ssh_ob = paramiko.SSHClient()
        # # ssh_ob.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        # ssh_ob.load_system_host_keys()
        # # ssh_ob.connect('GSP4-0077.local')
        # print("connecting to {}".format(hostname))
        # ssh_ob.connect(hostname, username=USER, password=PW)
        # file_path = file_dir + "/" + file_name
        # print("get file {}".format(file_path))
        # scp.put('sampletxt1.txt', 'sampletxt2.txt')

        self.scp.get(remote_path=LOG_DIR + file_name, local_path=".",recursive=False)
        # scp.put('sample', recursive=True, remote_path='/home/sample_user')

        # scp.close()
        # ssh_ob.close()
        return file_name

    def close(self):
        print("closing ssh, sftp, scp")
        self.sftp.close()
        self.ssh_ob.close()


if __name__ == "__main__":

    gf = Get_file("gsp4-0077.local")
    file_name = gf.get_calibration_file_name()
    gf.get_file(file_name=file_name)
    gf.close()

# gf.get_file("gsp4-0077.local","calibration-9.log","/home/augbooth/catkin_ws/local/roslogs/latest")