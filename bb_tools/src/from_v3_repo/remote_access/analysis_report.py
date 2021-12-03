import os
import sys
# Setup Django environment as remote access is outside Backoffice
# This file is only needed for Lionel
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "backoffice.settings")
cur_dir = os.path.dirname(os.path.abspath(__file__))
backoffice_dir = os.path.join(os.path.dirname(cur_dir), 'backoffice')
sys.path.append(backoffice_dir)
# End of Setup Django environment

import settings
from common.gs_settings import DEVICE_NAME
from controls.log_utils import get_analysis_summary

def get_summary():
    try:
        summary = get_analysis_summary()
    except ValueError as e:
        summary = 'Analysis cannot be retrieved! {}'.format(e.message)
    return summary

def send_summary(summary, location):
    """ Send the generated analysis summary via Dingtalk notif """

    # Get machine name
    machine_name = DEVICE_NAME
    # Alert on RemoteBot Chat
    curl_cmd = ("curl 'https://oapi.dingtalk.com/robot/send?access_token={0}' -H "
                "'Content-Type: application/json' "
                "-d '{{\"msgtype\":\"text\", \"text\": "
                "{{ \"content\":\"Analysis summary from {1} in {2}\n{3}\n{4}\n\" }}}}'").format(
                    settings.DINGTALK_ANALYSIS_TOKEN,
                    machine_name,
                    location,
                    "-" * 80, # Divider
                    summary
                )
    os.system(curl_cmd)
