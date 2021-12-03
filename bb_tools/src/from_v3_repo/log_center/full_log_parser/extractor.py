import os
import logging

import re
import math
from datetime import timedelta, datetime
from pathlib2 import Path
import pandas as pd
import numpy as np
import tempfile

import utils


LOGPATH = utils.log_path

logging.basicConfig(filename=LOGPATH, level=logging.DEBUG,
        format='%(asctime)s:%(levelname)s:%(message)s')


def generate_extracted_keywords(filedir, output_dir, tmpdir):
    """
    This method will output all lines that contain the specified keywords into a text file 
    """
    
    OUTPUT_FILE = utils.extracted_keywords_file
    files = [] #  Store only filenames we want to work on

    result = list(Path(filedir).glob('**/*'))

    if result == []:
        raise ValueError("Specified directory is Empty")
    
    desired_files  = "(" + ")|(".join(utils.desired_files_reg) + ")"
    
    for y in result:
        if os.path.isfile(str(y)):
            #  Look for files we want only
            if re.match(desired_files, str(y)):
                files.append(os.path.join(filedir, str(y)))   
    print("Identified relevant files to extract from")
    logging.info("Identified relevant files to extract from {}".format(files))
    combined = "(" + ")|(".join(utils.regex_list) + ")"

    write_file = open(os.path.join(tmpdir,OUTPUT_FILE),"w")

    for f in files:
        with open(f, "r") as fp:
            line = fp.readlines()
            for l in line:
                if l != '' and re.match(combined, l):
                    write_file.write(l)
    
    write_file_path = os.path.abspath(write_file.name)
    print("Storing misc files at : {}".format(tmpdir))
    write_file.close()
    logging.info("Writing to : {}".format(write_file_path))
    return convert_to_csv(write_file_path, output_dir, tmpdir)

def convert_to_csv(filepath, output_dir, tmpdir):
    """
    This method will convert the text file data into a CSV file, sorted by time in ascending order
    """
    logging.info("Converting to CSV file")
    if os.path.exists(filepath) is False:
        raise ValueError("Specified filepath for convert_to_csv is invalid")
    file_name = utils.sorted_extracted_file

    data = []
    with open(filepath) as f:
        for l in f.readlines():
            #  Extract out the relevant time and message portion
            t, msg = line_data(l)
            # Assert that values here are all within 5min of each other?

            # FIXME Comment out when using other logs   
            # if t < datetime.strptime("2019-08-20 09", "%Y-%m-%d %H"): # 9
            #     t += timedelta(hours=8)

            data.append({
                'time': t,
                'msg': msg
            })

    df = pd.DataFrame(data)

    df.set_index(keys="time", inplace=True)
    df.sort_index(inplace=True) #  Sort time
    df.to_csv(os.path.join(tmpdir, file_name),float_format='%g')
    # Uncomment if User wants to see in command line
    # print(os.path.join(tmpdir, file_name))
    return os.path.join(tmpdir, file_name)

def line_data(l):

    #  Extract out only time and message from the log files
    p = r'.*\[(ros.*)\]\[(?P<log_level>(INFO|DEBUG|ERROR|WARNING))\] (?P<time>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):(?P<msg>.*)'

    r = re.match(p, l)
    if r:
        return [
            datetime.strptime(r.group('time'), '%Y-%m-%d %H:%M:%S,%f'),
            r.group('msg').strip()
        ]
    logging.info("[ERROR] Unknown line {}".format(l))
    raise ValueError("Unknown line {}".format(l))

def gather_useful(df):
    """
    This method will identify only the valid portions of each log file and
    output a list containing the start time and end time of these time ranges
    """
    FMT = '%H:%M:%S.%f'
    logging.info("Gathering Useful Time from Sorted Extracted List")
    start_flag = False
    marked_flag = False
    end_flag = False
    start_time = ''
    start_date = ''
    end_time = ''
    end_date = ''
    useful_time_list = []
    sum_restart_duration = timedelta(microseconds=0)
    prev_start_time = None

    num_list = [0, 0, 0, 0] #  <fail, success, fail to mark, time restarting>
    num_failures = 0 #  Start, but fail to finish
    num_success = 0
    num_cannot_mark = 0
    for time, msg in df.itertuples(index=False):
        # validify_input(time, msg) # it must contain time accoridng to regex, 
        if time == "" and msg == "":
            continue
            
        s = re.match(utils.keyword_start, msg)
        e = re.match(utils.keyword_finish, msg)
        m = re.match(utils.keyword_marked, msg)

        date, time = time.split()
        
        if s:
            #  Goal Accepted
            logging.info("Goal Accepted Found")
            start_time = time
            start_date = date
            if prev_start_time == None:
                logging.info("No previous start time, initializing")
                prev_start_time = time

            if start_flag and marked_flag:
                logging.info("Start, Marked, NO END :{} {}".format(time, msg))

            if start_flag :
                #  Calculates the amount of time spent restarting
                sum_restart_duration += datetime.strptime(time, FMT) - datetime.strptime(prev_start_time,FMT)
                prev_start_time = time
                logging.info("Time taken to restart: {}".format(time))
                num_list[0] += 1
            start_flag = True    
        elif m:
            logging.info("marked true")
            marked_flag = True
        elif e:
            logging.info("end true")
            if (start_flag is True) and (end_flag is False):
                end_flag = True
                end_time = time
                end_date = date
                logging.info("\tend flag set true")
            
            if marked_flag is False and start_flag is False :
                logging.info("NO START, NO MARK, End : {} {}".format(time,msg))
                num_list[0] += 1
                start_flag = False
                end_flag = False
                marked_flag = False
                start_time = ''
                start_date = ''
                end_time = ''
                end_date = ''
                prev_start_time = None    
                continue

        #  Cannot mark point
        if start_flag and end_flag and (marked_flag is False):
            logging.info("Start, NO MARK, End : {} {}".format(time,msg))
            num_list[2] += 1
            start_flag = False
            end_flag = False
            start_time = '' 
            start_date = ''
            end_time = ''
            end_date = ''
            prev_start_time = None
            continue

        #  Successfully go to pt and marked
        if start_flag and end_flag and marked_flag:
            logging.info("start true, end true, marked true")
            logging.info("s_t, m_t, e_t {} {} {}".format(start_flag, marked_flag, end_flag))
            # Both True, good portion located
            useful_time_list.append((start_time, end_time, start_date, end_date))
            num_list[1] += 1
            #  Reset
            start_flag = False
            end_flag = False
            marked_flag = False
            start_time = ''
            start_date = ''
            end_time = ''
            end_date = ''
            prev_start_time = None
            continue
        num_list[3] = sum_restart_duration
        logging.info("Total time spent restarting: {}".format(sum_restart_duration))
    return useful_time_list, num_list
    
def remove_unwanted(df, output_dir, useful_time, tmpdir):
    """
    This method will remove all lines outside the valid time ranges and output
    csv file
    """
    logging.info("Removing unwanted")
    data = []
    file_name = utils.preprocessed_file
    for date_time, msg in df.itertuples(index=False):
        date, time = date_time.split()
        time_to_check = time
        date_to_check = date
      
        FMT = '%H:%M:%S.%f'
        FMT_date = "%Y-%m-%d"

        # FIXME Still not sure how to format this nicely..
        for start_time, end_time, start_date, end_date in useful_time:
            if ((datetime.strptime(start_time,FMT) <= datetime.strptime(time_to_check,FMT) <= datetime.strptime(end_time,FMT)) and
                (datetime.strptime(start_date,FMT_date) <= datetime.strptime(date_to_check,FMT_date) <= datetime.strptime(end_date,FMT_date))):
                data.append({
                    "time" : date_time,
                    "msg" : msg
                })
                break
    if data == []:
        raise ValueError("There is no useful time range!")
    good_data = pd.DataFrame(data) #  Remove the header and the left column
    good_data.set_index(keys="time", inplace=True)
    good_data.sort_index(inplace=True) #  Sort time
    good_data.to_csv(os.path.join(tmpdir,file_name), header=True, float_format='%g')

    return os.path.join(tmpdir, file_name)

def identify_exceptions(df):
    """ 
    Checks if the log files are in the desired sequence, if not, it will add it
    to a separate list and continue.
    """
    logging.info("identifying exceptions, find start sequence, order checking")
    keyword_start = r'(?P<keyword>nav: Goal accepted, id: )(?P<id>\d+)(, coords:) (?P<coord>\(-?\d+\.\d+, -?\d+\.\d+, -?\d+\.\d+\))'

    next_word = "first_entry"
    error_flag = False
    start_sequence_present = False
    exception_time_list = [] # Contains the timestamp of exceptions 
    exception_list = [] # Contains the start end of where the abnormal is
    goal_accepted_flag = False
    look_for_next_point = True
    timestamp = 'nothing'
    got_new_goal_flag = False

    for date_time, msg in df.itertuples(index=False):
        #  Find a start sequence
        if look_for_next_point:
            if re.match(keyword_start, msg):
                look_for_next_point = False
                start_sequence_present = True
            else:
                continue 
        if look_for_next_point is False:  
            error_flag, next_word, timestamp, got_new_goal_flag = order_checker(date_time, msg, next_word, error_flag, got_new_goal_flag)  
            if error_flag is True:
                #  Uncomment if user wants to see in cmd
                #print("Exception Identified: {} {}".format(date_time, msg))
                logging.info("Error flag true: {} {}".format(date_time, msg))
                exception_time_list.append(timestamp)
                look_for_next_point = True
                error_flag = False
                got_new_goal_flag = False
    
    if start_sequence_present == False:
        raise ValueError("No start point found")

    return exception_time_list

def remove_exception_from_useful_list(useful_time_list, exception_time_list):
    FMT = '%H:%M:%S.%f'
    FMT_date = "%Y-%m-%d"

    logging.info("Remove exception from useful list")
    assert useful_time_list != []
    exception_list = []
    for exception_date_time in exception_time_list:
        date, time = exception_date_time.split()
        time_to_check = time
        date_to_check = date
        # FIXME This formatting is very ugly
        for start_time, end_time, start_date, end_date in useful_time_list:
            if ((datetime.strptime(start_time,FMT) <= datetime.strptime(time_to_check,FMT) <= datetime.strptime(end_time,FMT)) and
                (datetime.strptime(start_date,FMT_date) <= datetime.strptime(date_to_check,FMT_date) <= datetime.strptime(end_date,FMT_date))):
                # Remove those with abnormal behaviour
                useful_time_list.remove((start_time, end_time, start_date, end_date))

                exception_list.append((start_time, end_time, start_date, end_date))
                break

    return useful_time_list, exception_list

def generate_exception_csv(df, output_dir, exception_list):
    """
    Takes creates a list of exceptions and output them into a csv
    """
    file_name = utils.exception_file
    data = create_exceptions_list_for_csv(df, exception_list)
    df_new = pd.DataFrame(data)
    df_new.set_index(keys="time", inplace=True)
    df_new.to_csv(os.path.join(output_dir,file_name), header=True, float_format='%g')
    exception_dict, total_duration_exception = analyze_exception_list(df_new)

    logging.info("Generating exception csv as at {}".format(os.path.join(output_dir,file_name)))
    
    return os.path.join(output_dir, file_name), exception_dict, total_duration_exception

def create_exceptions_list_for_csv(df, exception_list):
    """
    For those inside the range in exception_list, append to csv line by line
    separated by blank line
    """
    logging.info("Create exception list for csv")
    if exception_list == []:
        raise ValueError("Exception list provided is empty")

    FMT = '%H:%M:%S.%f'
    FMT_date = "%Y-%m-%d"
    appended_flag = False
    data = []
    for start_time, end_time, start_date, end_date in exception_list:

        for date_time, msg in df.itertuples(index=False):
            date, time = date_time.split()

            if((datetime.strptime(start_date,FMT_date) <= datetime.strptime(date,FMT_date) <=datetime.strptime(end_date,FMT_date)) and (datetime.strptime(start_time,FMT) <= datetime.strptime(time,FMT) <= datetime.strptime(end_time,FMT))):
                data.append({
                    "time" : date_time,
                    "msg" : msg
                })
                appended_flag = True
        if (appended_flag):
            data.append({
                "time" : "",
                "msg" : ""
            })
    #  If there are no exceptions, return empty
    if data == [] :
        raise ValueError("The exceptions cannot be found in the main list!")
    return data

def analyze_exception_list(df):
    """
    This list contains start and end, just that got some problems in between
    """
    logging.info("Analyzing exception list")
    identified = False
    gbm = 0
    next_regex = ['',''] #just placeholder

    total_duration = timedelta(microseconds=0)
    FMT = '%Y-%m-%d %H:%M:%S.%f'
    time_zero = datetime.strptime('00:00:00.0', '%H:%M:%S.%f')
    time = timedelta(microseconds=0)

    # Dictionary containing the diff errors and values
    breakdown_list = ({
        "gbm" : 0,
        "unknown" : 0,
        "unknown_id" : []
    })
    
    start = ''
    previous = ''
    for date_time, msg in df.itertuples():
        # add the timings
        if start == '':
            start = date_time
        if date_time == "":
            total_duration += datetime.strptime(previous, FMT) - datetime.strptime(start, FMT)
            start = ''
        previous = date_time

        if re.match(next_regex[0], msg) and next_regex != ['','']:
            # Access the key and add the value
            breakdown_list["gbm"] += 1
            identified = True
        next_regex = ['',''] #  Reset if the next is not match

        #  Leave commented for now (Until value is needed)
        #  TWO MOVEMENTS (check end > start)
        #  if re.match(utils.keyword_move_base_end, msg):
        #     next_regex = [utils.keyword_move_base_start, "move"]
        #  TWO GBM
        if re.match(utils.keyword_start_localisation, msg):
            next_regex = [utils.keyword_start_localisation, "gbm"]
        #  Identify unknown scenarios
        if re.match(utils.keyword_finish, msg) and identified is False:
            breakdown_list["unknown"] += 1
            breakdown_list["unknown_id"].append(msg.split()[2])
        if date_time == '' and msg == '':
            #  Reset
            identified = False
    return breakdown_list, total_duration

def order_checker(time, line, next_word, error_flag, got_new_goal_flag):
    """ 
    This method highlights any anomalous movement
    eg. move twice in a row, GBM actived twice in a row.
    Due to connection failure etc.
    """
    combined = "(" + ")|(".join(next_word) + ")"

    # Except for the first entry, no expected regex yet
    if next_word != 'first_entry':
        if re.match(combined, line) is None:
            return True, 'first_entry' , time, got_new_goal_flag

    # Goal accepted
    #  If the above condition is not fulfilled, it will try all below
    if re.match(utils.keyword_start, line):
        next_word = utils.expected_regex["keyword_start"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag

    # GBM Actived
    elif re.match(utils.keyword_start_localisation, line):
      
        if got_new_goal_flag:
            next_word = utils.expected_regex["keyword_start_localisation"]
            error_flag = False
            got_new_goal_flag = False
            return error_flag, next_word, '', got_new_goal_flag
        else:
            next_word = utils.expected_regex["keyword_start_localisation_no_goal"]
            error_flag = False
            return error_flag, next_word, '', got_new_goal_flag

    # Got new goal
    elif re.match(utils.keyword_tr_pos, line):
        got_new_goal_flag = True
        next_word = utils.expected_regex["keyword_tr_pos"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag

    # Move Base start
    elif re.match(utils.keyword_move_base_start, line):
        next_word = utils.expected_regex["keyword_move_base_start"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag        

    # move_base end
    elif re.match(utils.keyword_move_base_end, line):
        next_word = utils.expected_regex["keyword_move_base_end"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag
    
    # Localization
    elif re.match(utils.keyword_last_localisation, line):
        next_word = utils.expected_regex["keyword_last_localisation"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag     

    # Base estimate
    elif re.match(utils.keyword_end_localisation, line):
        next_word = utils.expected_regex["keyword_end_localisation"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag           

    # Marking
    elif re.match(utils.keyword_marked, line):
        next_word = utils.expected_regex["keyword_marked"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag             

    elif re.match(utils.keyword_finish, line):
        next_word = utils.expected_regex["keyword_finish"]
        error_flag = False
        return error_flag, next_word, '', got_new_goal_flag            

    else:
        logging.info("Order checker : Unknown Line {}".format(line))
        raise ValueError("Unknown line")

def calculate_total_localization_time(df):
    """ This method will calculate the total localisation time by parsing through all the rows in the respective localisation columns """
    
    logging.info("calculate total localization time")
    # Sum date time get total duration
    total_duration = timedelta(microseconds=0)
    FMT = '%H:%M:%S.%f'
    time_zero = datetime.strptime('00:00:00.0', '%H:%M:%S.%f')
    time = timedelta(microseconds=0)

    #  First Localisation
    for i in range(0, utils.MAX_NUM_STEPS): #  For 4 movements
        for line in df.iloc[
            :,(utils.LCSN_COL+i*(utils.NUM_VAR_FOR_EACH_MOVEMENT)
            )]:

            if (line!=line): #  Math.isnan raises some other error
                continue
            try:
                time = datetime.strptime(str(line), FMT) - time_zero
            except ValueError: #  CSV cuts the trailing zeroes.
                time_zero = datetime.strptime('00:00:00','%H:%M:%S')
                time = (datetime.strptime(str(line), '%H:%M:%S')) - time_zero
            finally:
                total_duration += time

    return total_duration #  In datetime format

def calculate_total_movement_time(df):
    """ 
    This method calculates the total movement time by parsing through each row
    of every movement column
    """
    logging.info("calculate total movement time")   
    # Sum date time get total duration
    total_duration = timedelta(microseconds=0)
    FMT = '%H:%M:%S.%f'
    time_zero = datetime.strptime('00:00:00.0', '%H:%M:%S.%f')
    time = timedelta(microseconds=0)

    #  First Localisation
    for i in range(0,4):
        for line in df.iloc[:, (utils.MOVETIME_COL+i*utils.NUM_VAR_FOR_EACH_MOVEMENT)]:#
            if (line!=line): # Same as L514
                continue
            try:
                time = datetime.strptime(str(line), FMT) - time_zero
            except ValueError: #  CSV cuts the trailing zeroes.
                time_zero = datetime.strptime('00:00:00','%H:%M:%S')
                time = (datetime.strptime(str(line), '%H:%M:%S')) - time_zero
            finally:
                total_duration += time
    return total_duration #  In datetime format

def calculate_total_duration(df, col):
    """
    This method will sum all the rows of the given column in datetime format
    """
    logging.info("calculate total duration")
    # Sum date time get total duration
    total_duration = timedelta(microseconds=0)
    FMT = '%H:%M:%S.%f'
    time_zero = datetime.strptime('00:00:00.0', '%H:%M:%S.%f')
    time = timedelta(microseconds=0)
    
    #  Loop through only the specified column
    for line in df.iloc[:, col]:
        if line!=line:  # See L514
            continue
        try:
            time = datetime.strptime(str(line), FMT) - time_zero
        except ValueError: 
            #  CSV cuts the trailing zeroes.
            time_zero = datetime.strptime('00:00:00','%H:%M:%S')
            time = (datetime.strptime(str(line), '%H:%M:%S')) - time_zero
        finally:
            total_duration += time
    return total_duration #  In datetime format

def filter_threshold(df, threshold):
    """
    This method will filter the formatted data based on the specified accuracy threshold
    """
    col_num = 0
    count = 0  
    #  List of accuracy columns for 4 steps
    col_list = [df.iloc[:, utils.ACC_COL], df.iloc[:, utils.ACC_COL*2], df.iloc[:, utils.ACC_COL*3], df.iloc[:, utils.ACC_COL*4]]

    #  It will loop through each ACCURACY col
    for col in col_list:
        count = 0
        for row in col:

            if math.isnan(row):
                count += 1
                continue
            #  Exceed 4 x threshold - Need localisation n movement
            elif (row >= threshold*4):
                pass
            #  Within 4x threshold - No localisation just movement
            elif (threshold <= row <= threshold*4): #FIXME magic numbers here
                df.iloc[count, 8+((col_num+1)*utils.NUM_VAR_FOR_EACH_MOVEMENT):utils.MOVEMENT_NUM_COL] = float('NaN')
            #  Less than threshold -Immediately stop
            elif (row <= threshold): # the first cell after each movement
                df.iloc[count, (utils.NUM_VAR_FOR_EACH_MOVEMENT + 1) + (col_num*utils.NUM_VAR_FOR_EACH_MOVEMENT):utils.MOVEMENT_NUM_COL] = float('NaN')
            count += 1
        col_num += 1
    return df

def generate_statistics(filepath):
    logging.info("Generate statistics")
    #  From static csv
    df = pd.read_csv(filepath, index_col=None, float_precision='high')
    time_lcsn = calculate_total_localization_time(df)
    time_movement = calculate_total_movement_time(df)
    time_mark = calculate_total_duration(df, utils.MARK_COL) 
    time_startend = calculate_total_duration(df, utils.STARTEND_COL)
    
    time_timecheck = calculate_total_duration(df, utils.TIMECHECK_COL)
    time_process_transition = time_startend - time_timecheck

    return time_lcsn, time_movement, time_mark, time_timecheck, time_startend 

def apply_threshold_on_statistics(filepath, threshold):
    """
    This will calculate the relevant values based on the filtered values via the specified threshold
    """
    
    df = pd.read_csv(filepath, index_col=None, float_precision='high')
    df_n = filter_threshold(df, threshold)
    time_lcsn = calculate_total_localization_time(df_n)       
    time_movement = calculate_total_movement_time(df_n)
    time_mark = calculate_total_duration(df_n, utils.MARK_COL)
    time_timecheck = time_movement + time_lcsn + time_mark
    return time_timecheck, time_lcsn, time_movement, time_mark

def compare_two_stats(result_1, result_2):
    """
    Takes in the return value from apply_threshold_on_statistics() and output
    the comparison data
    """

    check_1, lcsn_1, movement_1, mark_1 = result_1
    check_2, lcsn_2, movement_2, mark_2 = result_2

    total_time_saved_percentage = calc_time_saved_percentage(check_1, check_2)
    total_lcsn_saved_percentage = calc_time_saved_percentage(lcsn_1, lcsn_2)
    total_movement_saved_percentage = calc_time_saved_percentage(movement_1, movement_2)
    
    return total_time_saved_percentage, total_lcsn_saved_percentage,total_movement_saved_percentage
    
def calc_time_saved_percentage(value_1, value_2):
    return abs((1 - ((value_1.total_seconds()*1000)/(value_2.total_seconds()*1000) ))) * 100
