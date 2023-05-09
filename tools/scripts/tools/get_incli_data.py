import csv
import os
import re
import paramiko
# from paramiko import SSHClient
from scp import SCPClient
import sys

LOG_DIR = "/home/augbooth/catkin_ws/local/roslogs/latest/"
USER = "augbooth"
PW = "aug"
CHECK_LOG = "incli_check"

class Get_file():
    def __init__(self, hostname):
        print("connecting to {}".format(hostname))
        self.hostname = hostname
        self.ssh_ob = paramiko.SSHClient()
        self.ssh_ob.load_system_host_keys()
        self.ssh_ob.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh_ob.connect(hostname+".local", username=USER,password=PW)
        self.sftp = self.ssh_ob.open_sftp()
        self.scp = SCPClient(self.ssh_ob.get_transport())


    def get_inclination_file_name(self):
        for entry in self.sftp.listdir_attr(LOG_DIR):
            if entry.filename.startswith(CHECK_LOG) and "std" not in entry.filename:
                res = entry.filename
        return res

    def get_file(self, file_name):
        print("Getting file {}".format(file_name))
        self.scp.get(remote_path=LOG_DIR + file_name, local_path="./"+self.hostname+"_"+file_name,recursive=False)
        return self.hostname+"_"+file_name

    def close(self):
        print("closing ssh, sftp, scp")
        self.sftp.close()
        self.ssh_ob.close()

class GenCSV():
    def __init__(self,file_name):
        self.file_name = file_name
        self.raw_log_data = None
        pass


    def get_data(self, ):
        print("Using readlines()")
        with open(self.file_name) as fp:
            self.raw_log_data = fp.readlines()
        # return self.raw_log_data


    def gen_csv(self):
        with open(self.file_name[:-4]+".csv", 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile)
            # spamwriter.writerow(["test_id"])
            title_data = []
            title_data.append("servo_hor")
            title_data.append("servo_ver")
            title_data.append("rad_x")
            title_data.append("rad_y")
            spamwriter.writerow(title_data)   

            blank_row = [None,None,None,None]
            for l in self.raw_log_data:

                radian_and_laser_reg = "Got inclination: (.*, .*) at servos_radians: (.*,.*)"
                res = re.search(radian_and_laser_reg, l, re.M|re.I)
                if res is not None:
                    # print(l)
                    # print(res.groups()[0], res.groups()[1])
                    rad = res.groups()[0].replace("(", "")
                    rad = rad.replace(")", "")
                    rad_x = float(rad.split(",")[0])
                    rad_y = float(rad.split(",")[1])
                    print(rad_x,rad_y)
                    servo = res.groups()[1].replace("(", "")
                    servo = servo.replace(")", "")
                    servo_x = float(servo.split(",")[0])
                    servo_y = float(servo.split(",")[1])
                    print(servo_x, servo_y)
                    data = []
                    data.append(servo_x)
                    data.append(servo_y)
                    data.append(rad_x)
                    data.append(rad_y)
                    spamwriter.writerow(data)
                    print("-----")
                
            spamwriter.writerow(blank_row)
            spamwriter.writerow(blank_row)
            spamwriter.writerow(["difference(rad)",None,"=MAX(C2:C34)-MIN(C2:C34)","=MAX(D2:D34)-MIN(D2:D34)"])
            spamwriter.writerow(["average(rad)",None,"=AVERAGE(C2:C34)","=AVERAGE(D2:D34)"])
            spamwriter.writerow(blank_row)
            spamwriter.writerow(["difference(degree)",None,"=DEGREES(C37)","=DEGREES(D37)"])
            spamwriter.writerow(["average(degree)",None,"=DEGREES(C38)","=DEGREES(D38)"])


if __name__ == "__main__":
    args = sys.argv
    # print(args)
    if len(args) == 1:
        print("no param..exiting script.")
    else:
        hostname = args[1]
        print("Getting file from .... {}".format(hostname))
        gf = Get_file(hostname)
        fn = gf.get_inclination_file_name()
        log_file_name = gf.get_file(fn)

        gc = GenCSV(log_file_name)
        gc.get_data()
        gc.gen_csv()

    exit(0)

# gf = Get_file()
# gf.get_inclination_file_name()

file_name = "gsp4-0090-incli_check.log"

def get_data():
    print("Using readlines()")
    with open(file_name) as fp:
        raw_log_data = fp.readlines()
    return raw_log_data

raw_log_data =get_data()
with open(file_name[:-4]+".csv", 'w', newline='') as csvfile:

    spamwriter = csv.writer(csvfile)
    # spamwriter.writerow(["test_id"])
    title_data = []
    title_data.append("servo_hor")
    title_data.append("servo_ver")
    title_data.append("rad_x")
    title_data.append("rad_y")
    spamwriter.writerow(title_data)   

    blank_row = [None,None,None,None]
    for l in raw_log_data:

        radian_and_laser_reg = "Got inclination: (.*, .*) at servos_radians: (.*,.*)"
        res = re.search(radian_and_laser_reg, l, re.M|re.I)
        if res is not None:
            # print(l)
            # print(res.groups()[0], res.groups()[1])
            rad = res.groups()[0].replace("(", "")
            rad = rad.replace(")", "")
            rad_x = float(rad.split(",")[0])
            rad_y = float(rad.split(",")[1])
            print(rad_x,rad_y)
            servo = res.groups()[1].replace("(", "")
            servo = servo.replace(")", "")
            servo_x = float(servo.split(",")[0])
            servo_y = float(servo.split(",")[1])
            print(servo_x, servo_y)
            data = []
            data.append(servo_x)
            data.append(servo_y)
            data.append(rad_x)
            data.append(rad_y)
            spamwriter.writerow(data)
            print("-----")
        
    spamwriter.writerow(blank_row)
    spamwriter.writerow(blank_row)
    spamwriter.writerow(["difference(rad)",None,"=MAX(C2:C34)-MIN(C2:C34)","=MAX(D2:D34)-MIN(D2:D34)"])
    spamwriter.writerow(["average(rad)",None,"=AVERAGE(C2:C34)","=AVERAGE(D2:D34)"])
    spamwriter.writerow(blank_row)
    spamwriter.writerow(["difference(degree)",None,"=DEGREES(C37)","=DEGREES(D37)"])
    spamwriter.writerow(["average(degree)",None,"=DEGREES(C38)","=DEGREES(D38)"])