# GS laser test

### GS preparation and laser test

1. Power on GS.
2. Aim laser at the place that we want to measure.
3. ssh the GS, and follow the commnds:

```bash
cd catkin_ws/src/augustbot-tools
git pull
git checkout task-laser-on-gs-test
debug_shell
rosnode kill /measurement
```

5. Go to this directory and run the

```bash
cd ~/catkin_ws/src/augustbot-tools/tools/scripts/tools/gs_laser
python gs_laser.py
```

### laser data

6. If we want to end this test, quit this script `CTRL` + `c`

```bash
ls
```

7. In this diectory `~/catkin_ws/src/augustbot-tools/tools/scripts/tools/gs_laser` on GS, we see there is a json file named like `GSP4-0089_2023-05-11-06-38-41.json`
8. Send it to our computer via `scp`:

```bash
scp GSP4-0089_2023-05-11-06-38-41.json sunjilin@t14-0002.local:~
```

9. Copy this json to this diectory: `augustbot-tools/tools/scripts/tools/gs_laser`
10. Convert it to excel file, it will generate a excel about laser measurement.

```bash
cd augustbot-tools/tools/scripts/tools/gs_laser
python3 save_cvs.py
```
