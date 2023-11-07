import os
import csv
import yaml
import time
import sys
import shutil

yaml_data = None


if __name__ == "__main__":

    file_list = os.listdir()
    print(file_list)

    if not os.path.exists("tmp"):
        os.mkdir("tmp")

    yaml_list = []

    for f in file_list:
        if ".yaml" in f:
            yaml_list.append(f)


    for y in yaml_list:

        with open(y, "r") as f:
            yaml_data = yaml.safe_load(f)

        #print(yaml_data)


        for k,v in yaml_data.items():
            print("converting yaml to csv")
            with open(y[:-5]+"_"+k+".csv", "w") as f:
                cr = csv.writer(f)
                cr.writerow(["timestamp","servo_h","done","incli_x","incli_y"])
                for k1, v1 in v.items():
                    cr.writerow([k1,v1["servo"],v1["servo_done"],v1["inc_x"],v1["inc_y"]])
            
            time.sleep(3)
        
        shutil.move(y,"tmp")


