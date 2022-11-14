### gen_report.py
#### Notice
1. Please awared that the current version only works well for v3.4 or newer version (boothbot repo. on Lionel)
2. Make sure your pc has boothbot repo. on it (v3.4 or above)

#### Steps
1. Place all the interested rosout.log file (you can rename them, but they have to be in .log form) in the `CURRENT_DIR`
2. Excute the `gen_report.py` script

#### Outcome
1. The generated report will be store in `CURRENT_DIR/report_storage`
2. The processed report will be store in `CURRENT_DIR/roslog_storage`

### greb_logs_from_remote.py
#### Notice
1. The current version does not support "super-long" log (e.g. the rosout.log has been break down to rosout1.log, rosout2.log ..... )

#### Steps
1. Please make sure you have already `ssh-copy-id` your ssh `id_rsa.pub` to the machine you are going to be work with
2. run the script and follow the on screen instruction

#### Outcome
1. All the rosout.log will be copied to `CURRENT_DIR`