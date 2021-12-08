##### used in `lan_port_monitor.py` #####
BIG_PC_PORT = 'enp4s0'
SMALL_PC_PORT = 'enp3s0'
I5_PC_PORT = 'enp2s0'

# Gateway IP which need to del such that PC is able to
# access internet via router
# Not sure sometimes the gateway is different. BUT we just del all
# potential gateways
DEL_GW_IPS = ['169.254.43.99', '169.254.43.1']
#####

##### used in `remote_access.py` #####
DINGTALK_TOKEN = "535f7f44b98eab565061d87b2294797f0c55ca9da2a98b64850b4e1dad27d347"
#####

##### used in `analysis_report.py` #####
DINGTALK_ANALYSIS_TOKEN = "d3c481fc0fdaaf142fb36f3b8e97f1f47cbd0865ab2b1d0cf523d2c3c64bbec0"
#####

TOKEN_POOL = [
    # Below 10 tokens are from our `vpn1@a14s.com` ... `vpn10@a14s.com`
    '1cF9ijVEShx9aWk8ChXOfjLbHfK_6EMdtNAGu5GgW2hEf3xui', # vpn1
    '1cFBfbbOzJMWQ6rO80IzAcPJwSh_4LUwR5MXfWkuRE6N8jYwv', # vpn2
    '1cFBqzqZ9L1MWizhEvwPs8DZ1JY_6d1GSJnWnKnkiSRx36PKj', # vpn3
    '1cFBzGooORhIA9t9h9E8wHhcxth_6B4w52zBBShRPEcEj68E2', # vpn4
    '1cFC6j1dosDXZNfaflE8g617xoY_6hoBzPPgsY7dnvYpDuiFL', # vpn5
    '1cFCI1Re5FGIBkfQrz7DiHXbw3v_XeiXYJMHziWmk9aXBKRY',  # vpn6
    '1cFCNx7eQ1JF5y8OQfypiRijbMI_4Xiy28LaDd6ajhuMqjPZf', # vpn7
    '1cFCVwSnf1Z2x1GePKmzor3wub4_42wXppDts7jnp51NCn2C3', # vpn8
    '1cFCc9OeiNtc2UAeKs1Pmzq6Pm8_3NgcXcP8Z4FL5nST5Trsk', # vpn9
    '1cFCiKlWwQd2PTstJs5FHPdT3EF_6KMgYBFYbrsL9CBkLETUv', # vpn10

    # Below three tokens are from Max, Jay and Marshal
    '1VjRIGtNKF1KHZVSDsvbNOhKBul_5gSrQqc23bekLmVext4Uy', # Marshal
    '1VkBTXmatt217jpauu9HLWev6hk_51eYwbNPnguqFfJw72Jp8', # Max
    '1VkBHuKJwcIJPeRbc3YxwxoS7p4_4LQZPpgz7BpqufMWyfi1g', # Jay
]

REGION = "ap" # eu, us, ap

PING_SITE = "bing.com"
IS_CB = False

# Override exsiting settings to test code simpler.
try:
    from local_settings import *
except ImportError as e:
    pass
