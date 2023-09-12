import os
import json
import csv

file_list = os.listdir()

for f in file_list:
    if "json" not in f:
        continue

    if not f.lower().startswith("gsp"):
        continue
    
    print("handle {}".format(f))
    with open(f, "r") as f_d:
        data = json.load(f_d)

    with open(f[:-5]+".csv", "w") as c_f:
        writer = csv.writer(c_f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(["id","distance"])

        for k,v in data.items():
            writer.writerow([k,v])