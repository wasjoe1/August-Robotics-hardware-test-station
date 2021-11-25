import logging
import rospy

class ConnectPythonLoggingToROS(logging.Handler):
    """
    adopted from https://gist.github.com/nzjrs/8712011
    rospy.init_node('test_node', log_level=rospy.DEBUG)
    logging.basicConfig(format='%(message)')
    logging.getLogger('submodule').addHandler(ConnectPythonLoggingToROS())
    logging.getLogger('submodule').setLevel(logging.DEBUG)
    logger = logging.getLogger('submodule')
    logger.log(logging.INFO, "this is a test msg")
    logger.info(f"{msg}")
    """

    MAP = {
        logging.DEBUG:      rospy.logdebug,
        logging.INFO:       rospy.loginfo,
        logging.WARN:       rospy.logwarn,
        logging.ERROR:      rospy.logerr,
        logging.CRITICAL:   rospy.logfatal,
    }

    def emit(self, record):
        """
        record.name -> python logger name (e.g. 'test')
        record.msg  -> message after python logger formater
        """
        try:
            self.MAP[record.levelno](f"{record.name}: {record.msg}")
        except KeyError:
            rospy.logerr(f"Unknown log level {record.levelno} LOG: {record.name}: {record.msg}")

class CustomROSLogger(ConnectPythonLoggingToROS):
    
    def __init__(self, prefix, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.prefix = prefix

    def emit(self, record):
        try:
            self.MAP[record.levelno](f"{self.prefix}.{record.name}\t {record.msg}")
        except KeyError:
            rospy.logerr(f"{self.prefix}.{record.name}\t {record.msg}")

if __name__=="__main__":

    # try:
    #     from logger import CustomROSLogger
    #     ROSPY_SUPPORT = True
    # except ModuleNotFoundError:
    #     ROSPY_SUPPORT = False

    pass
