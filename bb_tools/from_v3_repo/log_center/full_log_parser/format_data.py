#!/usr/bin/env python
import os
import pandas as pd
import math
import re
import sys
from datetime import datetime, timedelta

import logging

import utils

LOGPATH = utils.log_path

class LogFileFormatter:

    def __init__(self, preprocessed_file, output_dir):

        logging.basicConfig(filename=LOGPATH, level=logging.DEBUG,
                        format='%(asctime)s:%(levelname)s:%(message)s')

################################### ATTRIBUTES ############################

        #  Read from input file
        self.filepath = preprocessed_file.filepath
        self.output_dir = output_dir
        self.df = pd.read_csv(self.filepath, index_col=False,float_precision='round_trip')
        #  Get total number of rows
        self.total_rows_count = len(self.df)

        self.output_file_name = utils.formatted_file
    
        #  Column format  (End Index of Range is temporary measure until I find a
        #  way to get the exact size)
        self.dfinal = pd.DataFrame(index=range(0, self.total_rows_count), columns=utils.table_col)   

         #  Time Variables
        self.local_start_time = ''
        self.local_end_time = ''
        self.local_x = ''
        self.local_y = ''
        self.local_rz = ''
        self.tr_x = ''
        self.tr_y = ''
        self.tr_rz = ''
        self.marking_start_time = ''
        self.marking_end_time = ''
        self.depart_time = '' #  Overall start time
        self.end_time = '' #  Overall end time
        self.move_start_time = ''
        self.move_end_time = ''
        self.time_at_last_movement = '' # Tracks the last movement time to get the time for non-localised move
        self.time_checker = timedelta(microseconds=0)
        self.time_between_points_marked = timedelta(microseconds=0)
        self.prev_end_time = None

        #  Other Counters
        self.row_count = 1 #  Ignore the header row
        self.movement = 0 #  Tracks the number of movements made
        self.point = 0
        self.consecutive_movement = 0

        #  Flags to keep track of status
        self.start_local = False #  If true, indicates Localization is starting
        self.start_find_local_end = False #???
        self.first_entry = True
        self.new_entry = True
        self.silent_flag = False #  If True, means perform last movement without localization
        self.base_estimate_pose_present = False
        self.marking_present = False
        self.get_final_pos_flag = False
        self.flipped_gbm = True

        #  Setup dataframe size
        self.data = []
        self.finaldata = [None]*utils.MAX_COL #  Can Refactor
           
        logging.info("init LogFileFormatter")
############################ FILTER METHODS #################################

    def get_start(self, line, date_time):
        """ Updates the Log Object with the Starting Time """
        date, time = date_time.split()
        self.depart_time = time
        self.new_entry = False
        self.point += 1
        self.finaldata = [None]*utils.MAX_COL
        self.finaldata[0] = str(self.point)

    def get_tr_pos(self,line):
        """ Update the Log Object with Lionel's next theoretical position """

        self.flipped_gbm = True
        self.consecutive_movement = 0
        line_split = re.split(r'[,\s()]', line)
        # FIXME refactor to split coord
        self.tr_x = line_split[len(line_split)-7]
        self.tr_y = line_split[len(line_split)-5]
        self.tr_rz = line_split[len(line_split)-1]
        
        # First_entry flag is to account for multiple movements in each journey    
        if (self.movement==0 and (self.first_entry is True)): 
            self.finaldata[utils.TR_X] = self.tr_x
            self.finaldata[utils.TR_Y] = self.tr_y
            self.finaldata[utils.TR_RZ] = self.tr_rz
            self.movement +=1 
            self.first_entry = False

        #  For subsequent movements within the same journey
        elif (self.first_entry is False):
            self.finaldata[utils.TR_X + (self.movement*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = self.tr_x
            self.finaldata[utils.TR_Y +(self.movement*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = self.tr_y
            self.finaldata[utils.TR_RZ +(self.movement*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = self.tr_rz
            self.movement += 1

    def get_start_move_time(self,line, date_time):
        line_split = re.split(r'[,\s()]', line)
        date, time = date_time.split()
        self.move_start_time = time

    def get_end_move_time(self, line, date_time):
        FMT = '%H:%M:%S.%f'
        line_split = re.split(r'[,\s()]', line)
        date, time = date_time.split()
        self.move_end_time = time
        self.time_at_last_movement = time
        self.start_local = True      

        #  Record move time
        tdelta = datetime.strptime(self.move_end_time, FMT) - datetime.strptime(self.move_start_time, FMT)
        self.time_checker += tdelta

        if self.consecutive_movement > 0:
            logging.info("second consecutive movement")
            #  For starting movement, and subsequent movement
            if (self.movement==1):
                logging.info("\tmovement==1 : {}".format(self.movement))
                new_tdelta = datetime.strptime(self.finaldata[utils.MOVETIME_COL], FMT) + tdelta
                self.finaldata[utils.MOVETIME_COL] = str(new_tdelta)[11:]

            else:
                logging.info("\tmovement!=1 : {}".format(self.movement))
                new_tdelta = datetime.strptime(self.finaldata[utils.MOVETIME_COL+((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)], FMT) + tdelta
                #  Then replace
                self.finaldata[utils.MOVETIME_COL+((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = str(new_tdelta)[11:]
            
        else:
            #  For starting movement, and subsequent movement
            if (self.movement==1):
                self.finaldata[utils.MOVETIME_COL] = str(tdelta)
            else:
                self.finaldata[utils.MOVETIME_COL+((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = str(tdelta)
        self.consecutive_movement += 1
    
    def get_localisation_start_time(self, line, date_time):
        self.silent_flag = False
        if self.flipped_gbm is True:
            logging.info("\tflipped gbm is true")
            self.movement -= 1  # -1 first
        if (self.start_local): #  If localisation indicator is true from the end of a movement
            line_split = re.split(r'[,\s()]', line)
            date, time = date_time.split()
            self.local_start_time = time #  Record the localisation start time
            # Debug
            self.start_local = False #  Reset

        # GBM after localisation
        elif (self.start_find_local_end): # End position
            if (self.movement == 1):
                self.record_intermediate_values()

            elif (self.movement>1):
                self.perform_no_lcsn_movement_above_one()
        
        if self.flipped_gbm:
            self.movement+=1
            self.flipped_gbm = False

    def perform_no_lcsn_movement_above_one(self):
        """ 
        Method to perform lcsn when movement is above one
        """
        coord = (self.local_x, self.local_y, self.local_rz)

        self.finaldata[utils.AC_X + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.local_x
        self.finaldata[utils.AC_Y + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.local_y
        self.finaldata[utils.AC_RZ + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.local_rz
        
        y = (utils.AC_Y + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)
        rz = (utils.AC_RZ + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)
        x = (utils.AC_X + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)

        #  Accuracy
        dist = math.hypot(float(self.local_x) - float(self.tr_x), float(self.local_y) - float(self.tr_y))
        
        self.finaldata[utils.ACC_COL + ((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = dist
        
        FMT = '%H:%M:%S.%f'
        
        tdelta = datetime.strptime(self.local_end_time, FMT) - datetime.strptime(self.local_start_time, FMT)
        
        self.time_checker += tdelta
        self.finaldata[utils.LCSN_COL + ((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = tdelta
        self.start_find_local_end = False

    #  Record_values_into_table
    def record_intermediate_values(self):
        logging.info("record intermediate values")
        #  Take the latest localisation data
        coord = (self.local_x, self.local_y, self.local_rz)
        
        self.finaldata[utils.AC_X] = self.local_x
        self.finaldata[utils.AC_Y] = self.local_y
        self.finaldata[utils.AC_RZ] = self.local_rz

        # Calculate accuracy
        dist = math.hypot(float(self.local_x) - float(self.tr_x), float(self.local_y) - float(self.tr_y))
        self.finaldata[utils.ACC_COL] = dist

        FMT = '%H:%M:%S.%f'
        #  Lcsn Time
        lcsnTime = datetime.strptime(self.local_end_time, FMT) - datetime.strptime(self.local_start_time, FMT)
        self.finaldata[utils.LCSN_COL] = lcsnTime
        self.time_checker += lcsnTime
        self.start_find_local_end = False

    def get_localisation_end_time(self, line, date_time):
        self.base_estimate_pose_present = True
        self.start_find_local_end = True
        line_split = re.split(r'[,\s()]', line)
        
        date, time = date_time.split()
        self.local_end_time = time
        self.local_x = line_split[len(line_split)-7]
        self.local_y = line_split[len(line_split)-5]
        self.local_rz = line_split[len(line_split)-2]
        self.silent_flag = True


    def get_end_time(self, line, date_time):
        date, time = date_time.split()
        self.marking_end_time = time
        self.end_time = time
        #  Track previous end time
        if self.prev_end_time is None:
            self.prev_end_time = time
        FMT = '%H:%M:%S.%f'

        #  Add marking time
        total_marking_time = datetime.strptime(self.marking_end_time, FMT) - datetime.strptime(self.marking_start_time, FMT)
        self.finaldata[utils.MARK_COL] = total_marking_time 
        
        self.time_checker += total_marking_time #  in ms             
        
        # Record time_checker
        self.finaldata[utils.TIMECHECK_COL] = self.time_checker
        self.time_checker = timedelta(microseconds=0) #  Reset

        # Calculate total Depart - End time
        self.total_journey_time = datetime.strptime(self.end_time, FMT) - datetime.strptime(self.depart_time, FMT)
        
        # Calculate time between each point marked
        if self.prev_end_time != None:
            time_difference_between_journey = datetime.strptime(self.depart_time, FMT) - datetime.strptime(self.prev_end_time, FMT)
            
            # Ignore those above 2 min time difference (Value averaged from
            # time between each point)
            if abs(time_difference_between_journey) < timedelta(minutes=2):
                self.time_between_points_marked += time_difference_between_journey
            self.prev_end_time = self.depart_time

        logging.info("Total Time between points: {}".format(self.time_between_points_marked))
        self.finaldata[utils.STARTEND_COL] = self.total_journey_time
        self.new_entry = True #  Release it in prep for next point

        #  Means never localised at last movement
        if (self.get_final_pos_flag is False):
            coord = (self.tr_x, self.tr_y, self.tr_rz)
            self.finaldata[utils.AC_X + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.tr_x
            self.finaldata[utils.AC_Y + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.tr_y
            self.finaldata[utils.AC_RZ + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.tr_rz
            self.finaldata[utils.MOVEMENT_NUM_COL] = self.movement
        else :
            #  Put localisation time
            tdelta = datetime.strptime(self.local_end_time, FMT) - datetime.strptime(self.local_start_time, FMT)
            self.finaldata[utils.LCSN_COL+((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = tdelta
            self.finaldata[utils.MOVEMENT_NUM_COL] = self.movement

        self.movement = 0
        self.dfinal.iloc[self.row_count] = self.finaldata
        self.row_count += 1
        self.first_entry = True
        self.get_final_pos_flag = False

    def get_final_pos(self): # need line as param?
        if (self.movement<1):
            logging.info("movement < 1 {}".format(self.movement))
            raise ValueError("Negative movement")

        self.get_final_pos_flag = True
        self.start_find_local_end = False
        coord = (self.local_x, self.local_y, self.local_rz) #  Final pos

        self.finaldata[utils.AC_X + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.local_x
        self.finaldata[utils.AC_Y + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.local_y
        self.finaldata[utils.AC_RZ + (self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT] = self.local_rz
        
        dist = math.hypot(float(self.local_x) - float(self.tr_x), float(self.local_y) - float(self.tr_y))
        
        self.finaldata[utils.ACC_COL + ((self.movement-1)*utils.NUM_VAR_FOR_EACH_MOVEMENT)] = dist
        
        FMT = '%H:%M:%S.%f'
        
        tdelta = datetime.strptime(self.local_end_time, FMT) - datetime.strptime(self.local_start_time, FMT)
        self.time_checker += tdelta

############################## FORMAT METHOD ################################

    def format(self, preprocessed_file, output_dir):
        """
        This method will iterate through each line and filter the results according
        to the keywords present

        Logic :
            1. Iterate through each line of the CSV file
            2. If the line contains any of the keywords
                    - Perform action, Store into new dataframe
                    - Store manually into their columns using indexing as values
                    obtained are not chronologous to the desired table format
            3. Because some keywords are repeated chronologically, flags are
            introduced on the first time the method is triggered to keep track
            of what actions to perform at which moment. These flags are resetted
            after each goal loop.
            4. Once all lines iterated through, store resultant data into CSV.
        """
        
        for date_time, line in self.df.itertuples(index=False):
            logging.info(line)

            #  Depart Time
            if re.match(utils.keyword_start, line):
                self.get_start(line, date_time)
                logging.info("movement: {}".format(self.movement))
                #print("start")

            # Theoretical Position
            elif re.match(utils.keyword_tr_pos, line):
                self.get_tr_pos(line)
                logging.info("movement: {}".format(self.movement))

            # Start Move Time
            elif re.match(utils.keyword_move_base_start, line):
                self.get_start_move_time(line, date_time)
                logging.info("movement: {}".format(self.movement))

            # End Move Time
            elif re.match(utils.keyword_move_base_end, line):
                self.get_end_move_time(line, date_time)
                logging.info("movement: {}".format(self.movement))

            # Start localisation time
            elif re.match(utils.keyword_start_localisation, line):
                self.get_localisation_start_time(line, date_time) 
                logging.info("movement: {}".format(self.movement))

            # Find Localisation End Time
            elif re.match(utils.keyword_end_localisation, line):
                self.get_localisation_end_time(line, date_time)
                logging.info("movement: {}".format(self.movement))
            # Find end of 1 journey (origin to desired point)
            
            elif re.match(utils.keyword_finish, line):
                self.get_end_time(line, date_time)
                logging.info("movement: {}".format(self.movement))

            # Last localisation
            elif re.match(utils.keyword_last_localisation, line):
                logging.info("movement: {}".format(self.movement))
                
                #  IF silent flag is TRUE
                if (self.silent_flag):
                    self.get_final_pos()

                # Get the time here
                line_split = re.split(r'[,\s()]', line)
                date, time = date_time.split()            
                self.marking_start_time = time #  Record start mark time
                
        self.dfinal.to_csv(os.path.join(output_dir, self.output_file_name), float_format='%g',index=False)

        if (self.base_estimate_pose_present is False):
            logging.error("Missing bare estimated pose")
            raise ValueError("ERROR: MISSING: 'Base estimated pose are'")

        logging.info("Total time between points marked: ".format(self.time_between_points_marked))
        
        return os.path.join(output_dir, self.output_file_name), self.time_between_points_marked

    def try_to_format(self):
        try :
            return self.format(self.filepath, self.output_dir)

        except pd.errors.EmptyDataError :
            logging.info("[ERROR] Empty data error")
            raise ValueError("ERROR: Log file is missing a keyword : 'Goal accepted, | finished '\nERROR: No processing will be done until all keywords are present")
        except ValueError as e:
            logging.error("ERROR: Log file is missing a keyword : 'GBM actived' | 'Got new goal' | 'move_base actived' | 'Move base done to' | 'Localization'\nERROR: No processing will be done until all keywords are present: {}".format(e))