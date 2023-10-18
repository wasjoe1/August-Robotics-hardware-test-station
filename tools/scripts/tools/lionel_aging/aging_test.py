import serial
import sys
import subprocess


def shell_cmd(cmd):
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, encoding='utf-8')
    stdout, stderr = p.communicate()
    return stdout, stderr


def enable_repeat_marking(args):
    if args[2].lower() == 'repeat':
        cmd = "rosservice call /drivers/db/srv_cmd \"command: \'REPEAT_MARKING:100\'\""
        stdout, stderr = shell_cmd(cmd)
        print(f'repeat marking accept status:{stdout},{stderr}')
    else:
        print('Will not enable repeat marking,since option is:'.format(args[2]))


def close_on_pump(args):
    ser = serial.Serial('/dev/painter', 115200)
    if args[1].lower() == 'on':
        ser.write('M221S100\n'.encode())
        print('turn on the pump')
    elif args[1].lower() == 'close':
        ser.write('M221S0\n'.encode())
        print('closed the pump')
    else:
        print(f'invalid option:{args[1]}\n'
              f'Will not change pump status')


def test():
    args = sys.argv
    if len(args) == 2:
        close_on_pump(args)
    elif len(args) == 1:
        print('Pls input the options,eg:python xx.py on/close/None repeat/None')
        sys.exit()
    elif len(args) == 3:
        close_on_pump(args)
        enable_repeat_marking(args)


if __name__ == '__main__':
    test()
