
## The principles of Script update

### Outputs

 - the time
 - the key date
 **explain:**
 Every script outputs in the yaml file which located in the Y_Doc with the name of CB29 or GS01 and so on. Record the time and the key date.
 Every Robot has one yaml file saved the production process and dates.

### Programming specification

- Just use the local launch file which only use driver (just like some program used in the bring_up_hardware.launch).
- Make the process simpler and more efficient.
- Have the necessary annotations.

### Document

- location:NASS:/lionel/06/02/Lionel-CB-production-manual-3.0.xls
- Pictures and description in Chinese and English
- Step by step

### Convenient

- Auto close the all opened windows after finished.
- Auto catch and deal with the date rather than manual record.

### Safety

- if someone run and stop the script on a good and completed robot,  make sure the robot will still work as usual.
- if has a wrong input it will be ok.
- it could be executed multiple times.

### Compatibility

- The script could be run on the boothbot computer and production computers. which means it could adapt to computer ros environment.for example : if the program is auto running or not ?


## Something need to do

### bug need to fix
- script: C0_initicial script
- phenomenon: it seem that GB go to the vertical zero point when script start. So if the vertical zero_offset (which in the device-settings) is not a appropriate value the GB will always try to go to the vertical- zero-point. and the control cmd is useless.

