import os
import logging
import re

import argparse
import errno
import pandas as pd
import tempfile
from datetime import timedelta, datetime

import extractor
import utils
from format_data import LogFileFormatter

LOGPATH = utils.log_path
logging.basicConfig(filename=LOGPATH, level=logging.DEBUG,
format='%(asctime)s:%(levelname)s:%(message)s')

class DataParser:
    """
    This class will be the main object through which all parsing activity is
    done, 1 DataParser will accommodate 1 entire log parsing
    """

    def __init__(self, filedir, output_filepath):
        self.filedir = filedir
        self.output_filepath = output_filepath
        self.num_list = []
        self.exception_list = []
        self.exception_dict = None
        self.tmpdir = tempfile.mkdtemp()
        self.total_duration_exception = timedelta(microseconds=0)
        self.time_between_points_marked = timedelta(microseconds=0)



    def swap_call_back_statements(self,df):
        cur_row = None
        prev_row = None
        cur_row_count = 0
        prev_row_count = -1
        counter = 0
        for time, msg in df.itertuples(index=False):
            cur_row = [time, msg]
            # cur_row_count += 1
            if prev_row == None:
                #print("prev row none")
                prev_row = cur_row
                cur_row_count += 1
                prev_row_count += 1
            else:
            #  If previous row matches "got new goal"
                if re.match(utils.keyword_tr_pos, prev_row[1]): 
                    #  If current row is Accepted and Next row is NOT Got New goal
                    if re.match(utils.keyword_start, cur_row[1]) and (re.match(utils.keyword_tr_pos, df.iat[cur_row_count+1,1]) is None):
                        counter += 1

                        #  Change the timing 
                        temp = df.iat[cur_row_count, 1]
                        df.iat[cur_row_count, 1] = df.iat[prev_row_count, 1]
                        df.iat[prev_row_count, 1] = temp
                        
                        cur_row_count +=1
                        prev_row_count += 1
                        #  Update previous row
                        prev_row = df.iat[cur_row_count, 0], df.iat[cur_row_count, 1]

                        continue
                    else:
                        
                        prev_row = cur_row
                        cur_row_count += 1
                        prev_row_count += 1 
                else:
                    prev_row = cur_row
                    prev_row_count += 1 
                    cur_row_count += 1

        logging.info("{} Accepted-GNG swapped".format(counter))
        return df

    def preprocess_data_file(self):
        """ 
        Takes in the directory containing log files to be processed
        """

        logging.info("Extracting")
        extracted_keywords = extractor.generate_extracted_keywords(self.filedir, self.output_filepath, self.tmpdir)

        df = pd.read_csv(extracted_keywords, float_precision='round_trip')
        
        #  Swap Goal Accepted and Got New Goal (if needed)
        #     Loop through DF, IF match Got new goal, and next is Goal
        #     accepted, SWITCH position
        #     Just loop through all and keep track of 2 rows, and row count
        df = self.swap_call_back_statements(df)

        #  Gather the start mark end ranges
        useful_time_list, self.num_list = extractor.gather_useful(df)

        if (len(useful_time_list) == 0):
            raise ValueError("ERROR: There is no valid time range")
            
        # Filter for Start, Mark, End
        filepath = extractor.remove_unwanted(df, self.output_filepath, useful_time_list, self.tmpdir)
        
        # Filter for abnormal cases
        df_2 = pd.read_csv(filepath, float_precision='round_trip')
        exception_list = extractor.identify_exceptions(df_2)
        #  If got exceptions, remove them
        if len(exception_list)>0:
            final_useful_time_list, self.exception_list = extractor.remove_exception_from_useful_list(useful_time_list, exception_list)
            # Remove exceptions from the useful time
            filepath = extractor.remove_unwanted(df_2,  self.output_filepath, final_useful_time_list, self.tmpdir)
            
            # Generate a csv containing the exception_list
            exception_csv_filepath, self.exception_dict, self.total_duration_exception = extractor.generate_exception_csv(df, self.output_filepath, self.exception_list)
            print("Detecting Exceptions...")
        preprocessed_file = PreprocessedFile(filepath, self.filedir)

        return preprocessed_file

    def format_data_file(self, preprocessed_file):
        """
        Takes in a preprocessed data file only
        """

        #  Only format if it is pre-processed
        if isinstance(preprocessed_file, PreprocessedFile):    
            # run my formatter
            fd = LogFileFormatter(preprocessed_file, self.output_filepath)
            formatted_file_path, self.time_between_points_marked = fd.try_to_format()
            print("Running Formatter...")
            return FormattedFile(formatted_file_path)

        raise ValueError("Trying to format non-preprocessed file {}"
            .format(preprocessed_file))

    def perform_analysis(self, formatted_file):
        """ 
        Takes in a formatted data file only and generates accuracy-time analysis
        """

        filepath = ''
        #  Only format if it is formatted
        if isinstance(formatted_file, FormattedFile):
            print("Running analysis")
            # Generate general stats
            time_lcsn, time_movement, time_mark, time_timecheck, time_startend = extractor.generate_statistics(formatted_file.filepath)
            #  Total time = Start-End for each point + Exception Time + Time
            #  spent restarting + Time between each point marked
            print(time_startend)
            print(self.total_duration_exception)
            print(self.num_list[3])
            print(self.time_between_points_marked)  
            total_time_elapsed = time_startend + self.total_duration_exception + self.num_list[3] + self.time_between_points_marked
            
            print("General Statistics:")
            print("Extracted from : {}".format(self.filedir))
            print("Total time elapsed: \t\t{}".format(total_time_elapsed))
            print("Number of points marked: \t{}".format(self.num_list[1]))
            print("Number of points fail to mark: \t{}".format(self.num_list[2]))
            print("Time from exceptions: \t\t{}".format(self.total_duration_exception + self.num_list[3]))
            print("Time between each point marked: {}".format(self.time_between_points_marked))
            print("Time between processes: \t{}".format(time_startend - time_timecheck))
            print("Total time of job done : \t{} (Excluding Exceptions and Time between processes)".format(time_timecheck))
            print("\tTotal localisation: \t{}".format(time_lcsn))
            print("\tTotal movement: \t{}".format(time_movement))
            print("\tTotal marking: \t\t{}".format(time_mark))
            print("Exceptions:")
            #  Keep following two statements now : Uncomment when needed
            print("\tStopped halfway and restart: {}".format(self.num_list[0]))
            if (self.exception_dict is not None):
                # print("\tMarked but moved twice in a row: {}".format(self.exception_dict["move"]))
                print("\tMarked but GBM twice in a row: {}".format(self.exception_dict["gbm"]))
                if (self.exception_dict["unknown"] != 0):
                    print("\tMarked but unknown scenario: {} (Goal: {})".format(self.exception_dict["unknown"], self.exception_dict["unknown_id"]))
            
            
            # UNCOMMENT IF YOU WANT TO COMPARE THE VALUES
            threshold_list = [0.005, 0.0075, 0.01, 0.02, 0.03]
            for threshold in threshold_list:
                timecheck, lcsn, movement, marking = extractor.apply_threshold_on_statistics(formatted_file.filepath, threshold)   
            #     print("Under {} Threshold:".format(threshold))
            #     print("Total time of processes: \t{}".format(timecheck))
            #     print("Total localisation: \t\t{}".format(lcsn))
            #     print("Total movement: \t\t{}".format(movement))
            #     print("Total marking: \t\t\t{}".format(marking))  
            # print('\n')
            #self.print_comparisons(threshold_list, formatted_file.filepath)
            

            return filepath # Currently not in use yet

        raise ValueError("Trying to format non-formatted file {}".format(formatted_file))

    def print_comparisons(self, threshold_list, filepath):
        """ Perform comparison and display """

        #  Validate Values
        if threshold_list == []:
            raise ValueError("Threshold list is empty! Nothing to compare")
        else:
            for num in threshold_list:
                if num <=0:
                    raise ValueError("Zero or Negative threshold")

        for i in range(0,len(threshold_list)):
            for j in range(i+1,len(threshold_list)):
                #  Get only values
                data_1 = extractor.apply_threshold_on_statistics(filepath, threshold_list[i])
                data_2 = extractor.apply_threshold_on_statistics(filepath, threshold_list[j])
                
                #  Get percentages
                total_time, total_lcsn, total_movement = extractor.compare_two_stats(data_1,data_2)
                
                process_diff = abs(data_1[0].total_seconds() - data_2[0].total_seconds())
                lcsn_diff = abs(data_1[1].total_seconds() - data_2[1].total_seconds())
                movement_diff = abs(data_1[2].total_seconds() - data_2[2].total_seconds())

                #  Print statement
                print("For {} to {}".format(threshold_list[i], threshold_list[j]))
                print("Time reduction:")
                print("Total Process Time: {}%, {}s".format(total_time, process_diff))
                print("Total Lcsn Time: {:0.3f}%, {}s".format(total_lcsn, lcsn_diff))
                print("Total Movement Time: {:0.3f}%, {}s".format(total_movement, movement_diff))
                
class PreprocessedFile:
    """
    This class represents a csv file that contains the information left
    after extracting only keyword related lines and only
    valid lines from the logfiles
    """ 

    def __init__(self, final_filepath, filedir):
        self.filepath = final_filepath
        self.filedir = filedir #  Directory containing the log file
        self.name = self.filepath.replace(self.filedir, "")

class FormattedFile:
    """
    This class represents a csv file that takes in a PreprocessedFile and
    formats it according to a table, breaking down each movement and timing of
    Lionel
    """

    def __init__(self, filepath):
        self.filepath = filepath
        self.name = filepath.replace(self.filepath,"")

class ProcessedFile:
    """
    This class represents a csv file of the final output after running a
    specified type of analyis method on the formatted file only
    """

    def __init__(self, filepath, analysis_type):
        self.filepath = filepath
        self.type = analysis_type #  Make proper enums in future for diff types
    
if __name__ == "__main__":

    parser = argparse.ArgumentParser(prog='python2 dataparser.py',
                                     usage= ('%(prog)s [-e <source dir> <output dir]\n\n'
                                             'Extract files from source dir and store output at output dir'
                                             ))

    parser.add_argument('-s', dest='source', required=True, default='')
    parser.add_argument('-o', dest='output', required=True, default='')
    
    args = parser.parse_args()
    source_filepath = args.source
    #  Verifying source
    if os.path.exists(source_filepath) is False:
        raise ValueError("Source directory does not exist")
   
    #  Verifying dest
    output_filepath = args.output
    if os.path.exists(output_filepath) is False:
        os.makedirs(output_filepath)

    print("Extracting from: {} \nOutputting at: {}".format(source_filepath, output_filepath))
    print("Logging at: {}".format(LOGPATH))
    dp = DataParser(source_filepath, output_filepath)
    
    logging.info("Attempting preprocess file from logs")
    #  Generate the data file from raw logs
    preprocessed_file = dp.preprocess_data_file()
    logging.info("Preprocess successful")
    logging.info("Attempting to format")
    ready_file = dp.format_data_file(preprocessed_file)
    logging.info("Formatted success")
    print("File ready to be processed")
    #  Select what kind of processing you want
    processed_file = dp.perform_analysis(ready_file)